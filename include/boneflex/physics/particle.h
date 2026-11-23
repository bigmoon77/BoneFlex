#pragma once

#include <boneflex/math/vec3.h>

namespace boneflex
{
    struct particle
    {
        vec3 position;
        vec3 previous_position;
        vec3 velocity;
        float inverse_mass = 1.0f;
    };
}
