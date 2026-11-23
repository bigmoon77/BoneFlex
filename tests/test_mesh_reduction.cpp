#include <boneflex/physics/mesh_reduction.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    bool test_grid_reduction_and_embedding()
    {
        std::vector<boneflex::vec3> positions;
        std::vector<std::uint32_t> triangles;
        constexpr std::uint32_t side = 24;
        for (std::uint32_t y = 0; y < side; ++y)
        {
            for (std::uint32_t x = 0; x < side; ++x)
            {
                positions.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
            }
        }
        for (std::uint32_t y = 0; y + 1 < side; ++y)
        {
            for (std::uint32_t x = 0; x + 1 < side; ++x)
            {
                const std::uint32_t a = y * side + x;
                const std::uint32_t b = a + 1;
                const std::uint32_t c = a + side;
                const std::uint32_t d = c + 1;
                triangles.insert(triangles.end(), {a, b, c, b, d, c});
            }
        }

        boneflex::mesh_reduction_options options;
        options.target_control_point_count = 64;
        options.embedding_influence_count = 4;
        boneflex::reduced_mesh reduced;
        boneflex::vertex_embedding embedding;
        if (!boneflex::generate_reduced_mesh(positions, triangles, options, reduced, embedding))
        {
            return false;
        }
        if (reduced.control_point_count() >= positions.size()
            || reduced.control_point_count() < 16
            || reduced.triangle_count() == 0
            || embedding.render_vertex_count() != positions.size())
        {
            return false;
        }

        std::vector<boneflex::vec3> reconstructed;
        if (!boneflex::reconstruct_render_vertices(reduced.rest_positions(), embedding, reconstructed)
            || reconstructed.size() != positions.size())
        {
            return false;
        }
        for (std::size_t i = 0; i < positions.size(); ++i)
        {
            if (!std::isfinite(reconstructed[i].x)
                || !std::isfinite(reconstructed[i].y)
                || !std::isfinite(reconstructed[i].z))
            {
                return false;
            }
        }
        return true;
    }

    bool test_invalid_input_preserves_outputs()
    {
        boneflex::reduced_mesh reduced;
        boneflex::vertex_embedding embedding;
        if (!reduced.set_data({{1.0f, 2.0f, 3.0f}}, {}))
        {
            return false;
        }
        if (!embedding.set_data(1, {0, 1}, {0}, {1.0f}, 1))
        {
            return false;
        }
        std::string error_message;
        boneflex::mesh_reduction_options options;
        const bool success = boneflex::generate_reduced_mesh({}, {}, options, reduced, embedding, &error_message);
        return !success && !error_message.empty()
            && reduced.control_point_count() == 1
            && embedding.render_vertex_count() == 1;
    }
}

int main()
{
    if (!test_grid_reduction_and_embedding())
    {
        std::cerr << "Grid reduction and embedding test failed.\n";
        return 1;
    }
    if (!test_invalid_input_preserves_outputs())
    {
        std::cerr << "Invalid input preservation test failed.\n";
        return 1;
    }
    std::cout << "All mesh reduction tests passed.\n";
    return 0;
}
