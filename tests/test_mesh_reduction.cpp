#include <boneflex/physics/mesh_reduction.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_set>
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

        std::vector<std::uint32_t> triangle_materials(triangles.size() / 3);
        for (std::size_t i = 0; i < triangle_materials.size(); ++i)
            triangle_materials[i] = static_cast<std::uint32_t>(i % 2);

        boneflex::mesh_reduction_options options;
        options.target_control_point_count = 64;
        options.embedding_influence_count = 4;
        boneflex::reduced_mesh reduced;
        boneflex::vertex_embedding embedding;
        if (!boneflex::generate_reduced_mesh(
            positions, triangles, options, reduced, embedding, nullptr, triangle_materials))
        {
            return false;
        }
        if (reduced.triangle_material_indices().size() != reduced.triangle_count())
            return false;
        bool has_material_zero = false;
        bool has_material_one = false;
        for (std::uint32_t material : reduced.triangle_material_indices())
        {
            has_material_zero = has_material_zero || material == 0;
            has_material_one = has_material_one || material == 1;
        }
        if (!has_material_zero || !has_material_one)
            return false;
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

    bool test_close_material_layers_remain_independent()
    {
        const std::vector<boneflex::vec3> positions{
            {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 0.02f}, {1.0f, 0.0f, 0.02f}, {0.0f, 1.0f, 0.02f}, {1.0f, 1.0f, 0.02f}
        };
        const std::vector<std::uint32_t> triangles{0, 1, 2, 1, 3, 2, 4, 5, 6, 5, 7, 6};
        const std::vector<std::uint32_t> materials{0, 0, 1, 1};
        boneflex::mesh_reduction_options options;
        options.target_control_point_count = 4;
        options.embedding_influence_count = 4;
        options.embedding_support_radius_scale = 3.0f;
        boneflex::reduced_mesh reduced;
        boneflex::vertex_embedding embedding;
        if (!boneflex::generate_reduced_mesh(
            positions, triangles, options, reduced, embedding, nullptr, materials))
            return false;

        std::unordered_set<std::uint32_t> material_zero_points;
        std::unordered_set<std::uint32_t> material_one_points;
        const auto& reduced_indices = reduced.triangle_indices();
        const auto& reduced_materials = reduced.triangle_material_indices();
        for (std::size_t triangle = 0; triangle < reduced_materials.size(); ++triangle)
        {
            auto& destination = reduced_materials[triangle] == 0
                ? material_zero_points : material_one_points;
            for (std::size_t corner = 0; corner < 3; ++corner)
                destination.insert(reduced_indices[triangle * 3 + corner]);
        }
        if (material_zero_points.empty() || material_one_points.empty())
            return false;
        for (const std::uint32_t point : material_zero_points)
            if (material_one_points.contains(point))
                return false;

        const auto& offsets = embedding.offsets();
        const auto& embedding_indices = embedding.control_point_indices();
        for (std::size_t vertex = 0; vertex < positions.size(); ++vertex)
        {
            const auto& allowed = vertex < 4 ? material_zero_points : material_one_points;
            for (std::uint32_t influence = offsets[vertex]; influence < offsets[vertex + 1]; ++influence)
                if (!allowed.contains(embedding_indices[influence]))
                    return false;
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
    if (!test_close_material_layers_remain_independent())
    {
        std::cerr << "Close material layer isolation test failed.\n";
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
