#include <boneflex/physics/reduced_soft_body_solver.h>
#include <boneflex/physics/reduced_mesh.h>
#include <boneflex/physics/shape_matching_cluster_generation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    using clock_type = std::chrono::steady_clock;

    double elapsed_ms(clock_type::time_point begin, clock_type::time_point end)
    {
        return std::chrono::duration<double, std::milli>(end - begin).count();
    }

    bool build_uv_sphere(std::uint32_t longitude_segments, std::uint32_t latitude_segments,
        boneflex::reduced_mesh& mesh)
    {
        std::vector<boneflex::vec3> positions;
        std::vector<std::uint32_t> indices;
        constexpr float pi = 3.14159265358979323846f;
        positions.reserve(static_cast<std::size_t>(longitude_segments) * latitude_segments);
        for (std::uint32_t y = 0; y < latitude_segments; ++y)
        {
            const float latitude = pi * static_cast<float>(y) / static_cast<float>(latitude_segments - 1);
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const float longitude = 2.0f * pi * static_cast<float>(x) / static_cast<float>(longitude_segments);
                positions.emplace_back(std::sin(latitude) * std::cos(longitude),
                    std::cos(latitude), std::sin(latitude) * std::sin(longitude));
            }
        }
        for (std::uint32_t y = 0; y + 1 < latitude_segments; ++y)
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const std::uint32_t next_x = (x + 1) % longitude_segments;
                const std::uint32_t a = y * longitude_segments + x;
                const std::uint32_t b = y * longitude_segments + next_x;
                const std::uint32_t c = (y + 1) * longitude_segments + x;
                const std::uint32_t d = (y + 1) * longitude_segments + next_x;
                indices.insert(indices.end(), {a, c, b, b, c, d});
            }
        return mesh.set_data(std::move(positions), std::move(indices));
    }
}

int main(int argc, char** argv)
{
    std::uint32_t longitude_segments = 200;
    std::uint32_t latitude_segments = 50;
    if (argc > 1) longitude_segments = static_cast<std::uint32_t>(std::stoul(argv[1]));
    if (argc > 2) latitude_segments = static_cast<std::uint32_t>(std::stoul(argv[2]));
    if (longitude_segments < 3 || latitude_segments < 3)
    {
        std::cerr << "Usage: boneflex_reduced_soft_body_benchmark [longitude_segments] [latitude_segments]\n";
        return 1;
    }

    boneflex::reduced_mesh mesh;
    if (!build_uv_sphere(longitude_segments, latitude_segments, mesh))
    {
        std::cerr << "Failed to create benchmark sphere.\n";
        return 1;
    }
    boneflex::shape_matching_cluster_options cluster_options;
    cluster_options.cluster_size = 32;
    boneflex::shape_matching_clusters clusters;
    std::string error_message;
    if (!boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters, &error_message))
    {
        std::cerr << "Cluster generation failed: " << error_message << '\n';
        return 1;
    }

    std::vector<boneflex::particle> particles(mesh.control_point_count());
    for (std::size_t i = 0; i < particles.size(); ++i)
    {
        particles[i].position = mesh.rest_positions()[i];
        particles[i].previous_position = particles[i].position;
    }
    boneflex::reduced_soft_body_options options;
    options.solver_iterations = 4;
    options.shape_matching_stiffness = 0.12f;
    options.velocity_damping = 0.995f;
    boneflex::reduced_soft_body_solver solver;
    if (!solver.initialize(particles, mesh.rest_positions(), {}, clusters, options))
    {
        std::cerr << "Solver initialization failed.\n";
        return 1;
    }

    constexpr std::size_t warmup_count = 5;
    constexpr std::size_t frame_count = 60;
    const float delta_time = 1.0f / 60.0f;
    for (std::size_t i = 0; i < warmup_count; ++i)
        if (!solver.step(delta_time)) return 1;

    const clock_type::time_point begin = clock_type::now();
    for (std::size_t i = 0; i < frame_count; ++i)
        if (!solver.step(delta_time)) return 1;
    const clock_type::time_point end = clock_type::now();

    double squared_displacement = 0.0;
    double maximum_displacement = 0.0;
    const std::vector<boneflex::particle>& result = solver.particles();
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        const double displacement = std::sqrt(static_cast<double>(
            (result[i].position - mesh.rest_positions()[i]).length_squared()));
        squared_displacement += displacement * displacement;
        maximum_displacement = std::max(maximum_displacement, displacement);
    }

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "BoneFlex reduced soft body benchmark\n";
    std::cout << "Control points:              " << solver.particle_count() << '\n';
    std::cout << "Shape matching clusters:     " << clusters.cluster_count() << '\n';
    std::cout << "Solver iterations:           " << options.solver_iterations << '\n';
    std::cout << "Average frame time:          " << elapsed_ms(begin, end) / frame_count << " ms\n";
    std::cout << "RMS displacement from rest:  "
        << std::sqrt(squared_displacement / static_cast<double>(result.size())) << '\n';
    std::cout << "Max displacement from rest:  " << maximum_displacement << '\n';
    return 0;
}
