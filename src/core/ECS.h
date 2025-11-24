#pragma once

#include <velox/Types.h>
#include <vector>
#include <unordered_map>
#include <typeindex>
#include <memory>
#include <cassert>
#include <algorithm>
#include <queue>
#include <array>

namespace Velox {

    // --- Component Traits ---
    using ComponentTypeID = std::uint32_t;

    inline ComponentTypeID GetUniqueComponentID() {
        static ComponentTypeID lastID = 0;
        return lastID++;
    }

    template <typename T>
    inline ComponentTypeID GetComponentTypeID() {
        static ComponentTypeID typeID = GetUniqueComponentID();
        return typeID;
    }

    // --- IComponentArray Interface ---
    class IComponentArray {
    public:
        virtual ~IComponentArray() = default;
        virtual void EntityDestroyed(EntityID entity) = 0;
    };

    // --- ComponentArray Implementation (Sparse Set) ---
    template <typename T>
    class ComponentArray : public IComponentArray {
    public:
        void InsertData(EntityID entity, T component) {
            size_t newIndex = m_size;
            m_entityToIndexMap[entity] = newIndex;
            m_indexToEntityMap[newIndex] = entity;
            m_componentArray[newIndex] = component;
            m_size++;
        }

        void RemoveData(EntityID entity) {
            if (m_entityToIndexMap.find(entity) == m_entityToIndexMap.end()) return;

            // Copy last element into deleted element's place to maintain density
            size_t removedIndex = m_entityToIndexMap[entity];
            size_t lastIndex = m_size - 1;
            T& lastComponent = m_componentArray[lastIndex];
            EntityID lastEntity = m_indexToEntityMap[lastIndex];

            m_componentArray[removedIndex] = lastComponent;
            m_entityToIndexMap[lastEntity] = removedIndex;
            m_indexToEntityMap[removedIndex] = lastEntity;

            m_entityToIndexMap.erase(entity);
            m_indexToEntityMap.erase(lastIndex);

            m_size--;
        }

        T& GetData(EntityID entity) {
            assert(m_entityToIndexMap.find(entity) != m_entityToIndexMap.end() && "Retrieving non-existent component.");
            return m_componentArray[m_entityToIndexMap[entity]];
        }

        bool HasData(EntityID entity) {
            return m_entityToIndexMap.find(entity) != m_entityToIndexMap.end();
        }

        void EntityDestroyed(EntityID entity) override {
            if (m_entityToIndexMap.find(entity) != m_entityToIndexMap.end()) {
                RemoveData(entity);
            }
        }

    private:
        // Packed array of components
        std::array<T, 10000> m_componentArray; // Fixed size for simplicity, can be dynamic
        
        // Map from EntityID to Index in m_componentArray
        std::unordered_map<EntityID, size_t> m_entityToIndexMap;
        
        // Map from Index to EntityID
        std::unordered_map<size_t, EntityID> m_indexToEntityMap;
        
        size_t m_size = 0;
    };

    // --- EntityManager ---
    class VELOX_API EntityManager {
    public:
        EntityManager() {
            // Initialize available entities
            for (EntityID i = 0; i < MAX_ENTITIES; ++i) {
                m_availableEntities.push(i);
            }
        }

        EntityID CreateEntity() {
            assert(m_livingEntityCount < MAX_ENTITIES && "Too many entities in existence.");
            EntityID id = m_availableEntities.front();
            m_availableEntities.pop();
            m_livingEntityCount++;
            return id;
        }

        void DestroyEntity(EntityID entity) {
            assert(entity < MAX_ENTITIES && "Entity out of range.");
            
            // Remove from all component arrays
            for (auto const& pair : m_componentArrays) {
                auto const& component = pair.second;
                component->EntityDestroyed(entity);
            }

            m_signatures[entity].clear();
            m_availableEntities.push(entity);
            m_livingEntityCount--;
        }

        template<typename T>
        void RegisterComponent() {
            const char* typeName = typeid(T).name();
            assert(m_componentTypes.find(typeName) == m_componentTypes.end() && "Registering component type more than once.");
            
            m_componentTypes.insert({typeName, GetComponentTypeID<T>()});
            m_componentArrays.insert({typeName, std::make_shared<ComponentArray<T>>()});
        }

        template<typename T>
        void AddComponent(EntityID entity, T component) {
            GetComponentArray<T>()->InsertData(entity, component);
        }

        template<typename T>
        void RemoveComponent(EntityID entity) {
            GetComponentArray<T>()->RemoveData(entity);
        }

        template<typename T>
        T& GetComponent(EntityID entity) {
            return GetComponentArray<T>()->GetData(entity);
        }
        
        template<typename T>
        bool HasComponent(EntityID entity) {
             return GetComponentArray<T>()->HasData(entity);
        }

        template<typename T>
        std::vector<EntityID> GetEntitiesWithComponent() {
             return {}; 
        }

    private:
        template<typename T>
        std::shared_ptr<ComponentArray<T>> GetComponentArray() {
            const char* typeName = typeid(T).name();
            assert(m_componentTypes.find(typeName) != m_componentTypes.end() && "Component not registered before use.");
            return std::static_pointer_cast<ComponentArray<T>>(m_componentArrays[typeName]);
        }

        static const EntityID MAX_ENTITIES = 10000;
        std::queue<EntityID> m_availableEntities;
        std::array<std::vector<ComponentTypeID>, MAX_ENTITIES> m_signatures; 
        uint32_t m_livingEntityCount = 0;

        std::unordered_map<const char*, ComponentTypeID> m_componentTypes;
        std::unordered_map<const char*, std::shared_ptr<IComponentArray>> m_componentArrays;
    };

}
