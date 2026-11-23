#include <boneflex/physics/reduced_mesh.h>
#include <boneflex/physics/shape_matching.h>
#include <boneflex/physics/shape_matching_cluster_generation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    using clock_type = std::chrono::steady_clock;

    double elapsed_milliseconds(clock_type::time_point begin, clock_type::time_point end)
    {
        return std::chrono::duration<double, std::milli>(end - begin).count();
    }

    bool build_uv_sphere(
        std::uint32_t longitude_segments,
        std::uint32_t latitude_segments,
        boneflex::reduced_mesh& output_mesh)
    {
        std::vector<boneflex::vec3> positions;
        std::vector<std::uint32_t> indices;
        positions.reserve(static_cast<std::size_t>(longitude_segments) * latitude_segments);
        constexpr float pi = 3.14159265358979323846f;

        for (std::uint32_t y = 0; y < latitude_segments; ++y)
        {
            const float latitude = pi * static_cast<float>(y)
                / static_cast<float>(latitude_segments - 1);
            const float sin_latitude = std::sin(latitude);
            const float cos_latitude = std::cos(latitude);
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const float longitude = 2.0f * pi * static_cast<float>(x)
                    / static_cast<float>(longitude_segments);
                positions.emplace_back(
                    sin_latitude * std::cos(longitude),
                    cos_latitude,
                    sin_latitude * std::sin(longitude));
            }
        }

        indices.reserve(static_cast<std::size_t>(longitude_segments)
            * (latitude_segments - 1) * 6);
        for (std::uint32_t y = 0; y + 1 < latitude_segments; ++y)
        {
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const std::uint32_t next_x = (x + 1) % longitude_segments;
                const std::uint32_t a = y * longitude_segments + x;
                const std::uint32_t b = y * longitude_segments + next_x;
                const std::uint32_t c = (y + 1) * longitude_segments + x;
                const std::uint32_t d = (y + 1) * longitude_segments + next_x;
                indices.insert(indices.end(), {a, c, b, b, c, d});
            }
        }
        return output_mesh.set_data(std::move(positions), std::move(indices));
    }
}

int main(int argc, char** argv)
{
    std::uint32_t longitude_segments = 200;
    std::uint32_t latitude_segments = 50;
    std::size_t cluster_size = 32;
    if (argc > 1) longitude_segments = static_cast<std::uint32_t>(std::stoul(argv[1]));
    if (argc > 2) latitude_segments = static_cast<std::uint32_t>(std::stoul(argv[2]));
    if (argc > 3) cluster_size = static_cast<std::size_t>(std::stoull(argv[3]));
    if (longitude_segments < 3 || latitude_segments < 3 || cluster_size < 4)
    {
        std::cerr << "Usage: boneflex_shape_matching_benchmark [longitude_segments] [latitude_segments] [cluster_size]\n";
        return 1;
    }

    boneflex::reduced_mesh mesh;
    if (!build_uv_sphere(longitude_segments, latitude_segments, mesh))
    {
        std::cerr << "Failed to build benchmark sphere.\n";
        return 1;
    }

    boneflex::shape_matching_cluster_options options;
    options.cluster_size = cluster_size;
    options.overlap_ratio = 0.5f;
    boneflex::shape_matching_clusters clusters;
    std::string error_message;
    const clock_type::time_point cluster_begin = clock_type::now();
    if (!boneflex::generate_shape_matching_clusters(mesh, options, clusters, &error_message))
    {
        std::cerr << "Cluster generation failed: " << error_message << '\n';
        return 1;
    }
    const clock_type::time_point cluster_end = clock_type::now();

    const std::vector<boneflex::vec3>& rest_positions = mesh.rest_positions();
    std::vector<boneflex::vec3> current_positions = rest_positions;
    for (std::size_t i = 0; i < current_positions.size(); ++i)
    {
        const float normalized_height = rest_positions[i].y;
        if (normalized_height > 0.80f)
        {
            const float falloff = (normalized_height - 0.80f) / 0.20f;
            current_positions[i].y += 0.30f * falloff * falloff;
        }
    }

    std::vector<boneflex::vec3> output_positions;
    constexpr std::size_t warmup_count = 2;
    constexpr std::size_t iteration_count = 10;
    for (std::size_t iteration = 0; iteration < warmup_count; ++iteration)
    {
        if (!boneflex::solve_shape_matching(
            rest_positions, current_positions, clusters, 0.85f, output_positions))
        {
            std::cerr << "Shape matching warmup failed.\n";
            return 1;
        }
    }

    const clock_type::time_point solve_begin = clock_type::now();
    for (std::size_t iteration = 0; iteration < iteration_count; ++iteration)
    {
        if (!boneflex::solve_shape_matching(
            rest_positions, current_positions, clusters, 0.85f, output_positions))
        {
            std::cerr << "Shape matching solve failed.\n";
            return 1;
        }
    }
    const clock_type::time_point solve_end = clock_type::now();

    double squared_displacement_sum = 0.0;
    double maximum_displacement = 0.0;
    for (std::size_t i = 0; i < output_positions.size(); ++i)
    {
        const double displacement = std::sqrt(static_cast<double>(
            (output_positions[i] - current_positions[i]).length_squared()));
        squared_displacement_sum += displacement * displacement;
        maximum_displacement = std::max(maximum_displacement, displacement);
    }
    const double rms_displacement = std::sqrt(
        squared_displacement_sum / static_cast<double>(output_positions.size()));

    const double average_solve_time = elapsed_milliseconds(solve_begin, solve_end)
        / static_cast<double>(iteration_count);
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "BoneFlex shape matching benchmark\n";
    std::cout << "Control points:           " << mesh.control_point_count() << '\n';
    std::cout << "Surface triangles:        " << mesh.triangle_count() << '\n';
    std::cout << "Cluster size:             " << cluster_size << '\n';
    std::cout << "Overlap ratio:            " << options.overlap_ratio << '\n';
    std::cout << "Generated clusters:       " << clusters.cluster_count() << '\n';
    std::cout << "Cluster memberships:      " << clusters.control_point_indices().size() << '\n';
    std::cout << "Cluster generation:       "
        << elapsed_milliseconds(cluster_begin, cluster_end) << " ms\n";
    std::cout << "Shape matching (average): " << average_solve_time << " ms\n";
    std::cout << "RMS correction:           " << rms_displacement << '\n';
    std::cout << "Max correction:           " << maximum_displacement << '\n';
    return 0;
}
