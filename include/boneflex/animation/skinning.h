#pragma once

#include <vector>

#include <boneflex/animation/skinning_weights.h>
#include <boneflex/math/transform.h>

namespace boneflex
{
    // CPU reference implementation. GPU backends can consume the same CSR weights
    // and skinning palette without changing the asset representation.
    bool skin_vertices(
        const std::vector<vec3>& bind_positions,
        const skinning_weights& weights,
        const std::vector<transform>& skinning_palette,
        std::vector<vec3>& output_positions
    );
}
