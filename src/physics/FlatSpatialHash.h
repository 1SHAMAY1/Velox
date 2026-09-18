#pragma once

#include "../math/Vec2.h"
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace Velox {

    struct SpatialGridCell {
        static constexpr int MAX_CELL_ENTITIES = 128;
        uint16_t count = 0;
        uint32_t entities[MAX_CELL_ENTITIES];
    };

    class FlatSpatialHash {
    public:
        FlatSpatialHash(Real cellSize = 32.0f, int tableSize = 8192)
            : m_cellSize(cellSize), m_invCellSize(1.0f / cellSize), m_tableSize(tableSize) {
            m_cells.resize(m_tableSize);
            m_cellOccupancy.reserve(2048);
        }

        void Clear() {
            for (uint32_t cellIdx : m_cellOccupancy) {
                m_cells[cellIdx].count = 0;
            }
            m_cellOccupancy.clear();
        }

        void SetCellSize(Real cellSize) {
            m_cellSize = cellSize;
            m_invCellSize = 1.0f / cellSize;
        }

        inline uint32_t Hash(int x, int y) const {
            // Fast 2D integer spatial hash (Prime multiplication)
            uint32_t h = (static_cast<uint32_t>(x) * 73856093u) ^ (static_cast<uint32_t>(y) * 19349663u);
            return h % static_cast<uint32_t>(m_tableSize);
        }

        void Insert(uint32_t entityId, const Vec2& min, const Vec2& max) {
            int minX = static_cast<int>(std::floor(min.x * m_invCellSize));
            int minY = static_cast<int>(std::floor(min.y * m_invCellSize));
            int maxX = static_cast<int>(std::floor(max.x * m_invCellSize));
            int maxY = static_cast<int>(std::floor(max.y * m_invCellSize));

            // Clamp max span to prevent pathological giant boxes from blowing up cells
            if (maxX - minX > 256) maxX = minX + 256;
            if (maxY - minY > 256) maxY = minY + 256;

            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    uint32_t cellIdx = Hash(x, y);
                    auto& cell = m_cells[cellIdx];
                    if (cell.count == 0) {
                        m_cellOccupancy.push_back(cellIdx);
                    }
                    if (cell.count < SpatialGridCell::MAX_CELL_ENTITIES) {
                        cell.entities[cell.count++] = entityId;
                    }
                }
            }
        }

        template <typename Callback>
        void Query(const Vec2& min, const Vec2& max, Callback&& callback) const {
            int minX = static_cast<int>(std::floor(min.x * m_invCellSize));
            int minY = static_cast<int>(std::floor(min.y * m_invCellSize));
            int maxX = static_cast<int>(std::floor(max.x * m_invCellSize));
            int maxY = static_cast<int>(std::floor(max.y * m_invCellSize));

            if (maxX - minX > 256) maxX = minX + 256;
            if (maxY - minY > 256) maxY = minY + 256;

            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    uint32_t cellIdx = Hash(x, y);
                    const auto& cell = m_cells[cellIdx];
                    for (uint32_t i = 0; i < cell.count; ++i) {
                        if (!callback(cell.entities[i])) {
                            return;
                        }
                    }
                }
            }
        }

    private:
        Real m_cellSize;
        Real m_invCellSize;
        int m_tableSize;
        std::vector<SpatialGridCell> m_cells;
        std::vector<uint32_t> m_cellOccupancy;
    };

} // namespace Velox
