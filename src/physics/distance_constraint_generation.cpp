#include <boneflex/physics/distance_constraint_generation.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>

namespace boneflex
{
    bool generate_distance_constraints(
        const reduced_mesh& mesh,
        const distance_constraint_generation_options& options,
        std::vector<xpbd_distance_constraint>& output_constraints,
        std::string* error_message)
    {
        const auto fail = [error_message](const char* message) -> bool
        {
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        };

        if (!std::isfinite(options.compliance) || options.compliance < 0.0f)
        {
            return fail("Distance constraint compliance must be finite and non-negative.");
        }
        if (mesh.control_point_count() == 0 || mesh.triangle_indices().size() % 3 != 0)
        {
            return fail("Distance constraints require a valid reduced mesh.");
        }

        const std::vector<vec3>& positions = mesh.rest_positions();
        const std::vector<std::uint32_t>& indices = mesh.triangle_indices();
        const auto& material_indices = mesh.triangle_material_indices();
        if (material_indices.size() != mesh.triangle_count()) return fail("Reduced mesh material data is invalid.");
        for (float value : options.material_compliances)
            if (!std::isfinite(value) || value < 0.0f) return fail("Material compliances must be finite and non-negative.");
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<float, std::size_t>> edge_compliances;

        for (std::size_t triangle_index = 0; triangle_index < indices.size(); triangle_index += 3)
        {
            const std::uint32_t triangle[3] = {
                indices[triangle_index],
                indices[triangle_index + 1],
                indices[triangle_index + 2]
            };
            for (std::size_t edge_index = 0; edge_index < 3; ++edge_index)
            {
                std::uint32_t a = triangle[edge_index];
                std::uint32_t b = triangle[(edge_index + 1) % 3];
                if (a >= positions.size() || b >= positions.size() || a == b)
                {
                    return fail("Reduced mesh contains an invalid triangle edge.");
                }
                if (a > b)
                {
                    std::swap(a, b);
                }
                const std::uint32_t material = material_indices[triangle_index / 3];
                if (!options.material_compliances.empty() && material >= options.material_compliances.size())
                    return fail("Triangle material index is outside the compliance table.");
                const float compliance = options.material_compliances.empty() ? options.compliance : options.material_compliances[material];
                auto& accumulated = edge_compliances[{a, b}];
                accumulated.first += compliance;
                ++accumulated.second;
            }
        }

        std::vector<xpbd_distance_constraint> result;
        result.reserve(edge_compliances.size());
        for (const auto& [edge, accumulated] : edge_compliances)
        {
            const float rest_length = (positions[edge.first] - positions[edge.second]).length();
            if (!std::isfinite(rest_length))
            {
                return fail("Reduced mesh produced a non-finite edge length.");
            }
            result.emplace_back(edge.first, edge.second, rest_length, accumulated.first / static_cast<float>(accumulated.second));
        }

        output_constraints = std::move(result);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }
}
