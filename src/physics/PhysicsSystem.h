#pragma once

#include "../core/VelcoxECS.h"
#include "Components.h"

namespace Velox {

    class PhysicsSystem {
    public:
        PhysicsSystem(std::shared_ptr<EntityManager> entityManager) 
            : m_entityManager(entityManager) {}

        void Step(Real dt);

    private:
        void ApplyRules();
        void Integrate(Real dt);
        void SolveConstraints(Real dt);

        std::shared_ptr<EntityManager> m_entityManager;

        // Spatial Grid Optimization
        struct SpatialGrid {
            static const int CELL_SIZE = 60; // Slightly larger than largest object (40-50)
            std::unordered_map<int, std::vector<EntityID>> cells;
            
            void Clear() { cells.clear(); }
            
            int GetHash(int x, int y) {
                // Simple hash for 2D grid
                return (x * 73856093) ^ (y * 19349663);
            }
            
            void Insert(EntityID id, const Vec2& min, const Vec2& max) {
                int startX = (int)min.x / CELL_SIZE;
                int endX = (int)max.x / CELL_SIZE;
                int startY = (int)min.y / CELL_SIZE;
                int endY = (int)max.y / CELL_SIZE;

                for (int x = startX; x <= endX; ++x) {
                    for (int y = startY; y <= endY; ++y) {
                        cells[GetHash(x, y)].push_back(id);
                    }
                }
            }
        } m_grid;

    };
}
