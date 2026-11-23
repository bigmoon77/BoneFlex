#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <boneflex/math/vec3.h>

namespace boneflex
{
    // Surface topology and the reduced control points used by the simulation.
    // Triangle indices reference control points, not render vertices.
    class reduced_mesh
    {
    public:
        bool set_data(
            std::vector<vec3> rest_positions,
            std::vector<std::uint32_t> triangle_indices,
            std::string* error_message = nullptr
        );

        std::size_t control_point_count() const;
        std::size_t triangle_count() const;
        const std::vector<vec3>& rest_positions() const;
        const std::vector<std::uint32_t>& triangle_indices() const;

    private:
        std::vector<vec3> _rest_positions;
        std::vector<std::uint32_t> _triangle_indices;
    };

    // CSR embedding from render vertices to reduced control points.
    // Each render vertex is reconstructed as a weighted sum of control points.
    class vertex_embedding
    {
    public:
        bool set_data(
            std::size_t render_vertex_count,
            std::vector<std::uint32_t> offsets,
            std::vector<std::uint32_t> control_point_indices,
            std::vector<float> weights,
            std::size_t control_point_count,
            std::string* error_message = nullptr
        );

        std::size_t render_vertex_count() const;
        std::size_t influence_count() const;
        const std::vector<std::uint32_t>& offsets() const;
        const std::vector<std::uint32_t>& control_point_indices() const;
        const std::vector<float>& weights() const;

    private:
        std::size_t _render_vertex_count = 0;
        std::vector<std::uint32_t> _offsets;
        std::vector<std::uint32_t> _control_point_indices;
        std::vector<float> _weights;
    };

    // CPU reference path for expanding reduced control-point positions.
    bool reconstruct_render_vertices(
        const std::vector<vec3>& control_point_positions,
        const vertex_embedding& embedding,
        std::vector<vec3>& output_positions
    );
}
