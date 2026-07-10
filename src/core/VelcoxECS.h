#pragma once

/**
 * @file VelcoxECS.h
 * @brief Sparse-set Entity Component System backing the Velox engine.
 *
 * Entities are plain integer handles. Components are stored densely per-type
 * in ComponentArray, with an EntityID <-> index map giving O(1) add/remove/lookup
 * while keeping iteration cache-friendly.
 */

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

    /// Issues a new, process-wide unique ComponentTypeID. Used only via GetComponentTypeID<T>().
    inline ComponentTypeID GetUniqueComponentID() {
        static ComponentTypeID lastID = 0;
        return lastID++;
    }

    /// Returns a stable ComponentTypeID for type T, assigned on first use.
    template <typename T>
    inline ComponentTypeID GetComponentTypeID() {
        static ComponentTypeID typeID = GetUniqueComponentID();
        return typeID;
    }

    /// Type-erased base so EntityManager can hold heterogeneous ComponentArray<T> instances.
    class IComponentArray {
    public:
        virtual ~IComponentArray() = default;

        /// Called by EntityManager when an entity is destroyed, so this array can drop its data.
        virtual void EntityDestroyed(EntityID entity) = 0;
    };

    /**
     * @brief Dense, packed storage for components of type T (sparse-set pattern).
     *
     * Components live contiguously in m_componentArray for fast iteration; two
     * maps translate between EntityID and array index. Removal swaps the last
     * element into the removed slot to keep the array dense in O(1).
     */
    template <typename T>
    class ComponentArray : public IComponentArray {
    public:
        /// Adds `component` for `entity`. Assumes the entity does not already have one.
        void InsertData(EntityID entity, T component) {
            size_t newIndex = m_size;
            m_entityToIndexMap[entity] = newIndex;
            m_indexToEntityMap[newIndex] = entity;
            m_componentArray[newIndex] = component;
            m_size++;
        }

        /// Removes `entity`'s component, if present, swapping the last element into its slot.
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

        /// Returns a reference to `entity`'s component. Asserts if it does not exist.
        T& GetData(EntityID entity) {
            assert(m_entityToIndexMap.find(entity) != m_entityToIndexMap.end() && "Retrieving non-existent component.");
            return m_componentArray[m_entityToIndexMap[entity]];
        }

        /// Returns true if `entity` currently has a component in this array.
        bool HasData(EntityID entity) {
            return m_entityToIndexMap.find(entity) != m_entityToIndexMap.end();
        }

        void EntityDestroyed(EntityID entity) override {
            if (m_entityToIndexMap.find(entity) != m_entityToIndexMap.end()) {
                RemoveData(entity);
            }
        }

        /// Number of components currently stored.
        size_t GetSize() const { return m_size; }

        /// Read-only access to the entity -> index map, used for iteration by owning entity.
        const std::unordered_map<EntityID, size_t>& GetEntityMap() const { return m_entityToIndexMap; }

    private:
        // Packed array of components
        std::array<T, 10000> m_componentArray; // Fixed size for simplicity, can be dynamic
        
        // Map from EntityID to Index in m_componentArray
        std::unordered_map<EntityID, size_t> m_entityToIndexMap;
        
        // Map from Index to EntityID
        std::unordered_map<size_t, EntityID> m_indexToEntityMap;
        
        size_t m_size = 0;
    };

    /**
     * @brief Owns all entities and their components for a single World.
     *
     * Entity IDs are recycled from a free list, and components are registered
     * per-type up front via RegisterComponent<T>() before they can be used.
     */
    class VELOX_API EntityManager {
    public:
        /// Seeds the free list with every entity ID up to MAX_ENTITIES.
        EntityManager() {
            for (EntityID i = 0; i < MAX_ENTITIES; ++i) {
                m_availableEntities.push(i);
            }
        }

        /// Allocates and returns a fresh EntityID from the free list.
        EntityID CreateEntity() {
            assert(m_livingEntityCount < MAX_ENTITIES && "Too many entities in existence.");
            EntityID id = m_availableEntities.front();
            m_availableEntities.pop();
            m_livingEntityCount++;
            return id;
        }

        /// Releases `entity` back to the pool and removes all of its components.
        void DestroyEntity(EntityID entity) {
            assert(entity < MAX_ENTITIES && "Entity out of range.");

            // Remove from all component arrays
            for (auto const& pair : m_componentArrays) {
                auto const& component = pair.second;
                component->EntityDestroyed(entity);
            }

            m_availableEntities.push(entity);
            m_livingEntityCount--;
        }

        /// Registers component type T, allocating its backing ComponentArray.
        /// Must be called once per type before Add/Remove/Get/HasComponent<T>() are used.
        template<typename T>
        void RegisterComponent() {
            const char* typeName = typeid(T).name();
            assert(m_componentTypes.find(typeName) == m_componentTypes.end() && "Registering component type more than once.");

            m_componentTypes.insert({typeName, GetComponentTypeID<T>()});
            m_componentArrays.insert({typeName, std::make_shared<ComponentArray<T>>()});
        }

        /// Attaches a component of type T to `entity`.
        template<typename T>
        void AddComponent(EntityID entity, T component) {
            GetComponentArray<T>()->InsertData(entity, component);
        }

        /// Detaches the component of type T from `entity`, if present.
        template<typename T>
        void RemoveComponent(EntityID entity) {
            GetComponentArray<T>()->RemoveData(entity);
        }

        /// Returns a mutable reference to `entity`'s component of type T.
        template<typename T>
        T& GetComponent(EntityID entity) {
            return GetComponentArray<T>()->GetData(entity);
        }

        /// Returns true if `entity` has a component of type T.
        template<typename T>
        bool HasComponent(EntityID entity) {
             return GetComponentArray<T>()->HasData(entity);
        }

        /// Returns the IDs of every entity currently holding a component of type T.
        template<typename T>
        std::vector<EntityID> GetEntitiesWithComponent() {
            auto array = GetComponentArray<T>();
            std::vector<EntityID> entities;
            entities.reserve(array->GetSize());
            for (const auto& pair : array->GetEntityMap()) {
                entities.push_back(pair.first);
            }
            return entities;
        }

    private:
        /// Looks up (and type-casts) the ComponentArray registered for type T.
        template<typename T>
        std::shared_ptr<ComponentArray<T>> GetComponentArray() {
            const char* typeName = typeid(T).name();
            assert(m_componentTypes.find(typeName) != m_componentTypes.end() && "Component not registered before use.");
            return std::static_pointer_cast<ComponentArray<T>>(m_componentArrays[typeName]);
        }
        static const EntityID MAX_ENTITIES = 10000; ///< Upper bound on simultaneously live entities.
        std::queue<EntityID> m_availableEntities;    ///< Free list of recyclable entity IDs.
        uint32_t m_livingEntityCount = 0;            ///< Number of entities currently allocated.

        std::unordered_map<const char*, ComponentTypeID> m_componentTypes;             ///< Registered component type IDs, keyed by RTTI name.
        std::unordered_map<const char*, std::shared_ptr<IComponentArray>> m_componentArrays; ///< Backing storage per registered component type.
    };

}