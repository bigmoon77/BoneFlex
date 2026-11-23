#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <boneflex/animation/skeleton.h>
#include <boneflex/animation/skinning.h>

namespace boneflex
{
    // Data exported by the BoneFlex Blender add-on. Mesh positions are in
    // armature-local space with the Blender Z-up to BoneFlex Y-up conversion
    // already applied.
    struct blender_asset
    {
        std::vector<vec3> bind_positions;
        std::vector<std::uint32_t> triangle_indices;
        skeleton rig;
        skinning_weights weights;

        bool skin_current_pose(std::vector<vec3>& output_positions) const;
    };

    // Loads the versioned, little-endian .bflex interchange file.
    bool load_blender_asset(
        const std::string& file_path,
        blender_asset& output_asset,
        std::string* error_message = nullptr
    );
}
