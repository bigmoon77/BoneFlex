#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <boneflex/math/vec3.h>

namespace boneflex
{
    // CSR list of control points participating in each shape-matching cluster.
    class shape_matching_clusters
    {
    public:
        bool set_data(
            std::size_t control_point_count,
            std::vector<std::uint32_t> offsets,
            std::vector<std::uint32_t> control_point_indices,
            std::string* error_message = nullptr
        );

        std::size_t cluster_count() const;
        std::size_t control_point_count() const;
        const std::vector<std::uint32_t>& offsets() const;
        const std::vector<std::uint32_t>& control_point_indices() const;

    private:
        std::size_t _control_point_count = 0;
        std::vector<std::uint32_t> _offsets;
        std::vector<std::uint32_t> _control_point_indices;
    };

    // CPU reference implementation of overlapping rigid shape matching.
    // Each cluster computes a best-fit rigid transform; overlapping goals are averaged.
    bool solve_shape_matching(
        const std::vector<vec3>& rest_positions,
        const std::vector<vec3>& current_positions,
        const shape_matching_clusters& clusters,
        float stiffness,
        std::vector<vec3>& output_positions
    );

    // Fixed points determine the cluster translation while all points determine rotation.
    // This prevents unanchored deformation from translating an anchored cluster as a whole.
    bool solve_shape_matching(
        const std::vector<vec3>& rest_positions,
        const std::vector<vec3>& current_positions,
        const shape_matching_clusters& clusters,
        const std::vector<float>& fixed_point_weights,
        float stiffness,
        std::vector<vec3>& output_positions
    );
}
