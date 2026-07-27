#pragma once

#include "../core/Containers.h"
#include <cstdint>

namespace Velox {

// ============================================================================
// IslandManager: Zero-STL Disjoint Set Union (DSU) & Sleep Graph
// ============================================================================
class IslandManager {
public:
    IslandManager() : m_entityCount(0) {}

    void Initialize(size_t maxEntities) {
        m_entityCount = maxEntities;
        m_parent.Resize(maxEntities);
        m_rank.Resize(maxEntities);
        m_islandEnergy.Resize(maxEntities);
        m_islandBodyCount.Resize(maxEntities);
        m_isSleepingIsland.Resize(maxEntities);
        Reset();
    }

    void Reset() {
        for (size_t i = 0; i < m_entityCount; ++i) {
            m_parent[i] = static_cast<int32_t>(i);
            m_rank[i] = 0;
            m_islandEnergy[i] = 0.0f;
            m_islandBodyCount[i] = 1;
            m_isSleepingIsland[i] = 0;
        }
    }

    int32_t Find(int32_t x) {
        if (x < 0 || static_cast<size_t>(x) >= m_entityCount) return x;
        int32_t root = x;
        while (root != m_parent[root]) {
            root = m_parent[root];
        }
        // Path compression
        int32_t curr = x;
        while (curr != root) {
            int32_t next = m_parent[curr];
            m_parent[curr] = root;
            curr = next;
        }
        return root;
    }

    void Union(int32_t a, int32_t b) {
        int32_t rootA = Find(a);
        int32_t rootB = Find(b);
        if (rootA == rootB) return;

        if (m_rank[rootA] < m_rank[rootB]) {
            m_parent[rootA] = rootB;
            m_islandBodyCount[rootB] += m_islandBodyCount[rootA];
        } else if (m_rank[rootA] > m_rank[rootB]) {
            m_parent[rootB] = rootA;
            m_islandBodyCount[rootA] += m_islandBodyCount[rootB];
        } else {
            m_parent[rootB] = rootA;
            m_islandBodyCount[rootA] += m_islandBodyCount[rootB];
            m_rank[rootA]++;
        }
    }

    void AddKineticEnergy(int32_t entityId, float energy) {
        int32_t root = Find(entityId);
        if (root >= 0 && static_cast<size_t>(root) < m_entityCount) {
            m_islandEnergy[root] += energy;
        }
    }

    bool IsIslandSettled(int32_t entityId, float energyThresholdPerBody = 0.02f) {
        int32_t root = Find(entityId);
        if (root < 0 || static_cast<size_t>(root) >= m_entityCount) return false;
        int32_t count = m_islandBodyCount[root];
        if (count <= 0) return true;
        float avgEnergy = m_islandEnergy[root] / static_cast<float>(count);
        return avgEnergy < energyThresholdPerBody;
    }

private:
    size_t m_entityCount;
    PodVector<int32_t> m_parent;
    PodVector<int32_t> m_rank;
    PodVector<float> m_islandEnergy;
    PodVector<int32_t> m_islandBodyCount;
    PodVector<uint8_t> m_isSleepingIsland;
};

} // namespace Velox
