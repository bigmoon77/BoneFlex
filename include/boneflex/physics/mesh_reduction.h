#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <boneflex/math/vec3.h>
#include <boneflex/physics/reduced_mesh.h>

namespace boneflex
{
    struct mesh_reduction_options
    {
        std::size_t target_control_point_count = 5000;
        // Minimum number of local influences when the support is sparse.
        std::size_t embedding_influence_count = 4;
        // Compact-kernel support radius measured in voxel cell widths.
        float embedding_support_radius_scale = 2.75f;
    };

    // Builds a deterministic voxel-clustered surface mesh and a local
    // compact-kernel embedding from the original render vertices.
    bool generate_reduced_mesh(
        const std::vector<vec3>& render_positions,
        const std::vector<std::uint32_t>& render_triangle_indices,
        const mesh_reduction_options& options,
        reduced_mesh& output_mesh,
        vertex_embedding& output_embedding,
        std::string* error_message = nullptr,
        const std::vector<std::uint32_t>& render_triangle_material_indices = {}
    );
}
