#include <boneflex/physics/distance_constraint_generation.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
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
        std::set<std::pair<std::uint32_t, std::uint32_t>> unique_edges;

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
                unique_edges.emplace(a, b);
            }
        }

        std::vector<xpbd_distance_constraint> result;
        result.reserve(unique_edges.size());
        for (const std::pair<std::uint32_t, std::uint32_t>& edge : unique_edges)
        {
            const float rest_length = (positions[edge.first] - positions[edge.second]).length();
            if (!std::isfinite(rest_length))
            {
                return fail("Reduced mesh produced a non-finite edge length.");
            }
            result.emplace_back(edge.first, edge.second, rest_length, options.compliance);
        }

        output_constraints = std::move(result);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }
}
