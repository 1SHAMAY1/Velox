#pragma once

/**
 * @file Types.h
 * @brief Fundamental type aliases and export macros shared across the Velox engine.
 */

#include <cstdint>
#include <limits>
#include <cmath>

#ifdef WIN32
    #ifdef VELOX_EXPORTS
        #define VELOX_API __declspec(dllexport) ///< Marks a symbol for export when building the Velox DLL.
    #else
        #define VELOX_API __declspec(dllimport) ///< Marks a symbol for import when consuming the Velox DLL.
    #endif
#else
    #define VELOX_API ///< No-op on platforms that don't require import/export decoration.
#endif

namespace Velox {

    /// Unique handle identifying an entity within an EntityManager.
    using EntityID = uint32_t;

    /// Unique identifier assigned to each registered component type.
    using ComponentTypeID = uint32_t;

    /// Sentinel value representing "no entity" / an invalid handle.
    constexpr EntityID INVALID_ENTITY = std::numeric_limits<EntityID>::max();

    /// Scalar precision used throughout the math and physics code.
    /// Switch to `double` here if higher precision is required.
    using Real = float;

}