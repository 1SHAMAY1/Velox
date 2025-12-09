#pragma once

#include <cstdint>
#include <limits>
#include <cmath>

#ifdef WIN32
    #ifdef VELOX_EXPORTS
        #define VELOX_API __declspec(dllexport)
    #else
        #define VELOX_API __declspec(dllimport)
    #endif
#else
    #define VELOX_API
#endif

namespace Velox {

    using EntityID = uint32_t;
    using ComponentTypeID = uint32_t;

    constexpr EntityID INVALID_ENTITY = std::numeric_limits<EntityID>::max();

    // Basic scalar types
    using Real = float; // Can be switched to double for higher precision

}
