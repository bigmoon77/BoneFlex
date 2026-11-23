#pragma once

#include <cstdint>
#include <string>

#include <boneflex/math/transform.h>

namespace boneflex
{
    struct bone
    {
        static constexpr std::int32_t root_parent = -1;

        std::string name;
        std::int32_t parent_index = root_parent;
        transform bind_local_pose;
    };
}
