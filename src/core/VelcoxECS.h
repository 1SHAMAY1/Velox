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
        static constexpr size_t INVALID_INDEX = static_cast<size_t>(-1);

        ComponentArray() {
            m_sparseEntityMap.resize(128, INVALID_INDEX);
            m_componentArray.reserve(32);
            m_denseEntities.reserve(32);
        }

        /// Adds `component` for `entity`. Assumes the entity does not already have one.
        void InsertData(EntityID entity, T component) {
            if (entity >= m_sparseEntityMap.size()) {
                m_sparseEntityMap.resize(std::max(m_sparseEntityMap.size() * 2, static_cast<size_t>(entity + 128)), INVALID_INDEX);
            }
            size_t newIndex = m_size;
            m_sparseEntityMap[entity] = newIndex;
            m_denseEntities.push_back(entity);
            if (newIndex < m_componentArray.size()) {
                m_componentArray[newIndex] = component;
            } else {
                m_componentArray.push_back(component);
            }
            m_size++;
        }

        /// Removes `entity`'s component, if present, swapping the last element into its slot.
        void RemoveData(EntityID entity) {
            if (entity >= m_sparseEntityMap.size() || m_sparseEntityMap[entity] == INVALID_INDEX) return;

            size_t removedIndex = m_sparseEntityMap[entity];
            size_t lastIndex = m_size - 1;

            if (removedIndex != lastIndex) {
                T lastComponent = m_componentArray[lastIndex];
                EntityID lastEntity = m_denseEntities[lastIndex];

                m_componentArray[removedIndex] = lastComponent;
                m_sparseEntityMap[lastEntity] = removedIndex;
                m_denseEntities[removedIndex] = lastEntity;
            }

            m_sparseEntityMap[entity] = INVALID_INDEX;
            m_denseEntities.pop_back();
            m_size--;
        }

        /// Returns a reference to `entity`'s component. Asserts if it does not exist.
        inline T& GetData(EntityID entity) {
            assert(entity < m_sparseEntityMap.size() && m_sparseEntityMap[entity] != INVALID_INDEX && "Retrieving non-existent component.");
            return m_componentArray[m_sparseEntityMap[entity]];
        }

        /// Returns a const reference to `entity`'s component.
        inline const T& GetData(EntityID entity) const {
            assert(entity < m_sparseEntityMap.size() && m_sparseEntityMap[entity] != INVALID_INDEX && "Retrieving non-existent component.");
            return m_componentArray[m_sparseEntityMap[entity]];
        }

        /// Returns true if `entity` currently has a component in this array.
        inline bool HasData(EntityID entity) const {
            return entity < m_sparseEntityMap.size() && m_sparseEntityMap[entity] != INVALID_INDEX;
        }

        void EntityDestroyed(EntityID entity) override {
            if (entity < m_sparseEntityMap.size() && m_sparseEntityMap[entity] != INVALID_INDEX) {
                RemoveData(entity);
            }
        }

        /// Number of components currently stored.
        size_t GetSize() const { return m_size; }

        /// Direct const ref to dense entities for zero-allocation iteration
        const std::vector<EntityID>& GetDenseEntities() const { return m_denseEntities; }
        const std::vector<T>& GetComponentArray() const { return m_componentArray; }
        std::vector<T>& GetComponentArray() { return m_componentArray; }

    private:
        std::vector<T> m_componentArray;
        std::vector<size_t> m_sparseEntityMap;
        std::vector<EntityID> m_denseEntities;
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
            std::type_index typeIndex(typeid(T));
            assert(m_componentTypes.find(typeIndex) == m_componentTypes.end() && "Registering component type more than once.");

            ComponentTypeID typeID = GetComponentTypeID<T>();
            m_componentTypes.insert({typeIndex, typeID});
            
            auto arr = std::make_shared<ComponentArray<T>>();
            m_componentArrays.insert({typeIndex, arr});

            if (typeID >= m_fastArrayLookup.size()) {
                m_fastArrayLookup.resize(typeID + 32, nullptr);
            }
            m_fastArrayLookup[typeID] = arr.get();
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
        inline T& GetComponent(EntityID entity) {
            ComponentTypeID typeID = GetComponentTypeID<T>();
            assert(typeID < m_fastArrayLookup.size() && m_fastArrayLookup[typeID] != nullptr && "Component not registered before use.");
            return static_cast<ComponentArray<T>*>(m_fastArrayLookup[typeID])->GetData(entity);
        }

        /// Returns true if component type T has been registered in this EntityManager.
        template<typename T>
        inline bool HasComponentType() const {
            ComponentTypeID typeID = GetComponentTypeID<T>();
            return typeID < m_fastArrayLookup.size() && m_fastArrayLookup[typeID] != nullptr;
        }

        /// Returns true if `entity` has a component of type T.
        template<typename T>
        inline bool HasComponent(EntityID entity) const {
            ComponentTypeID typeID = GetComponentTypeID<T>();
            if (typeID >= m_fastArrayLookup.size() || !m_fastArrayLookup[typeID]) return false;
            return static_cast<const ComponentArray<T>*>(m_fastArrayLookup[typeID])->HasData(entity);
        }

        /// Returns the IDs of every entity currently holding a component of type T (zero-copy const ref).
        /// If component type T is not registered in this world, safely returns an empty vector.
        template<typename T>
        inline const std::vector<EntityID>& GetEntitiesWithComponent() const {
            ComponentTypeID typeID = GetComponentTypeID<T>();
            if (typeID >= m_fastArrayLookup.size() || !m_fastArrayLookup[typeID]) {
                static const std::vector<EntityID> emptyEntities;
                return emptyEntities;
            }
            return static_cast<const ComponentArray<T>*>(m_fastArrayLookup[typeID])->GetDenseEntities();
        }

        /// Returns direct pointer to contiguous ComponentArray<T> for high-throughput solver loops.
        template<typename T>
        inline ComponentArray<T>* GetComponentArrayFast() {
            ComponentTypeID typeID = GetComponentTypeID<T>();
            if (typeID < m_fastArrayLookup.size()) {
                return static_cast<ComponentArray<T>*>(m_fastArrayLookup[typeID]);
            }
            return nullptr;
        }

    private:
        /// Looks up (and type-casts) the ComponentArray registered for type T.
        template<typename T>
        std::shared_ptr<ComponentArray<T>> GetComponentArray() {
            std::type_index typeIndex(typeid(T));
            assert(m_componentTypes.find(typeIndex) != m_componentTypes.end() && "Component not registered before use.");
            return std::static_pointer_cast<ComponentArray<T>>(m_componentArrays[typeIndex]);
        }
        static const EntityID MAX_ENTITIES = 10000; ///< Upper bound on simultaneously live entities.
        std::queue<EntityID> m_availableEntities;    ///< Free list of recyclable entity IDs.
        uint32_t m_livingEntityCount = 0;            ///< Number of entities currently allocated.

        std::unordered_map<std::type_index, ComponentTypeID> m_componentTypes;             ///< Registered component type IDs, keyed by std::type_index.
        std::unordered_map<std::type_index, std::shared_ptr<IComponentArray>> m_componentArrays; ///< Backing storage per registered component type.
        std::vector<IComponentArray*> m_fastArrayLookup; ///< Flat O(1) direct pointer array indexed by ComponentTypeID.
    };

}