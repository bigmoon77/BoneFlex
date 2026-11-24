#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <boneflex/animation/skeleton.h>
#include <boneflex/animation/skinning.h>

namespace boneflex
{
    struct blender_material_physics
    {
        std::string name;
        float shape_matching_stiffness = 0.15f;
        float xpbd_compliance = 1.0e-5f;
        float velocity_damping = 0.995f;
        float max_deformation_ratio = 0.5f;
    };

    // Data exported by the BoneFlex Blender add-on. Mesh positions are in
    // armature-local space with the Blender Z-up to BoneFlex Y-up conversion
    // already applied.
    struct blender_asset
    {
        std::vector<vec3> bind_positions;
        std::vector<std::uint32_t> triangle_indices;
        // One material slot index per triangle. Version 1 assets use slot zero.
        std::vector<std::uint32_t> triangle_material_indices;
        std::vector<blender_material_physics> materials;
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
