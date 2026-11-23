#include <boneflex/physics/shape_matching.h>
#include <boneflex/physics/shape_matching_cluster_generation.h>
#include <boneflex/physics/reduced_mesh.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

namespace
{
    bool test_local_deformation_propagates_smoothly()
    {
        constexpr std::uint32_t side = 16;
        std::vector<boneflex::vec3> rest_positions;
        std::vector<std::uint32_t> triangle_indices;
        rest_positions.reserve(side * side);

        for (std::uint32_t y = 0; y < side; ++y)
        {
            for (std::uint32_t x = 0; x < side; ++x)
            {
                rest_positions.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
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
                triangle_indices.insert(triangle_indices.end(), {a, b, c, b, d, c});
            }
        }

        boneflex::reduced_mesh mesh;
        if (!mesh.set_data(rest_positions, std::move(triangle_indices)))
        {
            return false;
        }

        boneflex::shape_matching_cluster_options options;
        options.cluster_size = 32;
        options.overlap_ratio = 0.5f;

        boneflex::shape_matching_clusters clusters;
        if (!boneflex::generate_shape_matching_clusters(mesh, options, clusters))
        {
            return false;
        }

        std::vector<boneflex::vec3> current_positions = rest_positions;
        constexpr std::uint32_t dragged_x = side / 2;
        constexpr std::uint32_t dragged_y = side / 2;
        const std::size_t dragged_index = dragged_y * side + dragged_x;
        current_positions[dragged_index].z = 2.0f;

        std::vector<boneflex::vec3> zero_stiffness_output;
        if (!boneflex::solve_shape_matching(
            rest_positions, current_positions, clusters, 0.0f, zero_stiffness_output))
        {
            return false;
        }
        for (std::size_t i = 0; i < current_positions.size(); ++i)
        {
            if ((zero_stiffness_output[i] - current_positions[i]).length() > 1.0e-6f)
            {
                return false;
            }
        }

        std::vector<boneflex::vec3> output_positions;
        if (!boneflex::solve_shape_matching(
            rest_positions, current_positions, clusters, 1.0f, output_positions))
        {
            return false;
        }

        if (!(output_positions[dragged_index].z > 0.0f
            && output_positions[dragged_index].z < current_positions[dragged_index].z))
        {
            return false;
        }

        float maximum_neighbor_displacement = 0.0f;
        for (std::uint32_t y = dragged_y - 2; y <= dragged_y + 2; ++y)
        {
            for (std::uint32_t x = dragged_x - 2; x <= dragged_x + 2; ++x)
            {
                if (x == dragged_x && y == dragged_y)
                {
                    continue;
                }
                const std::size_t index = y * side + x;
                maximum_neighbor_displacement = std::max(
                    maximum_neighbor_displacement, std::abs(output_positions[index].z));
            }
        }
        if (maximum_neighbor_displacement < 1.0e-4f)
        {
            std::cerr << "Local deformation did not propagate to neighboring control points.\n";
            return false;
        }

        const std::size_t distant_index = 1 * side + 1;
        if (std::abs(output_positions[distant_index].z) > 1.0e-4f)
        {
            std::cerr << "Local deformation affected a distant control point.\n";
            return false;
        }
        return true;
    }
}

int main()
{
    if (!test_local_deformation_propagates_smoothly())
    {
        std::cerr << "Shape matching deformation test failed.\n";
        return 1;
    }
    std::cout << "All shape matching deformation tests passed.\n";
    return 0;
}
