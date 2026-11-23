#include <boneflex/physics/reduced_soft_body_solver.h>
#include <boneflex/physics/distance_constraint_generation.h>
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

    struct displacement_metrics
    {
        std::size_t count = 0;
        double mean = 0.0;
        double rms = 0.0;
        double maximum = 0.0;
        double mean_pull_axis = 0.0;
    };

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

    displacement_metrics measure_displacement(
        const std::vector<boneflex::vec3>& rest_positions,
        const std::vector<boneflex::vec3>& current_positions,
        const std::vector<std::size_t>& indices)
    {
        displacement_metrics metrics;
        metrics.count = indices.size();
        if (indices.empty())
        {
            return metrics;
        }

        double sum = 0.0;
        double squared_sum = 0.0;
        double pull_axis_sum = 0.0;
        for (std::size_t index : indices)
        {
            const boneflex::vec3 displacement = current_positions[index] - rest_positions[index];
            const double magnitude = std::sqrt(static_cast<double>(displacement.length_squared()));
            sum += magnitude;
            squared_sum += magnitude * magnitude;
            pull_axis_sum += displacement.x;
            metrics.maximum = std::max(metrics.maximum, magnitude);
        }
        metrics.mean = sum / static_cast<double>(metrics.count);
        metrics.rms = std::sqrt(squared_sum / static_cast<double>(metrics.count));
        metrics.mean_pull_axis = pull_axis_sum / static_cast<double>(metrics.count);
        return metrics;
    }

    std::vector<boneflex::vec3> capture_positions(const std::vector<boneflex::particle>& particles)
    {
        std::vector<boneflex::vec3> positions;
        positions.reserve(particles.size());
        for (const boneflex::particle& value : particles)
        {
            positions.push_back(value.position);
        }
        return positions;
    }

    void print_region(const char* name, const displacement_metrics& metrics)
    {
        std::cout << name << " count: " << metrics.count
            << ", mean: " << metrics.mean
            << ", RMS: " << metrics.rms
            << ", max: " << metrics.maximum
            << ", mean X: " << metrics.mean_pull_axis << '\n';
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
    std::vector<std::size_t> pull_region;
    std::vector<std::size_t> neighbor_region;
    std::vector<std::size_t> far_region;
    std::vector<boneflex::vec3> pull_accelerations(mesh.control_point_count());
    for (std::size_t i = 0; i < particles.size(); ++i)
    {
        particles[i].position = mesh.rest_positions()[i];
        particles[i].previous_position = particles[i].position;

        const float x = mesh.rest_positions()[i].x;
        if (x >= 0.90f)
        {
            pull_region.push_back(i);
        }
        else if (x >= 0.50f)
        {
            neighbor_region.push_back(i);
        }
        else if (x <= 0.0f)
        {
            far_region.push_back(i);
        }

        const float falloff = std::clamp((x - 0.65f) / 0.35f, 0.0f, 1.0f);
        const float smooth_falloff = falloff * falloff * (3.0f - 2.0f * falloff);
        pull_accelerations[i] = boneflex::vec3(12.0f * smooth_falloff, 0.0f, 0.0f);
    }

    boneflex::distance_constraint_generation_options constraint_options;
    constraint_options.compliance = 1.0e-6f;
    std::vector<boneflex::xpbd_distance_constraint> distance_constraints;
    if (!boneflex::generate_distance_constraints(mesh, constraint_options, distance_constraints, &error_message))
    {
        std::cerr << "Distance constraint generation failed: " << error_message << '\n';
        return 1;
    }

    boneflex::reduced_soft_body_options options;
    options.solver_iterations = 4;
    options.shape_matching_stiffness = 0.12f;
    options.gravity = boneflex::vec3();
    options.velocity_damping = 0.995f;

    boneflex::reduced_soft_body_solver solver;
    if (!solver.initialize(particles, mesh.rest_positions(), distance_constraints, clusters, options))
    {
        std::cerr << "Solver initialization failed.\n";
        return 1;
    }

    constexpr std::size_t warmup_count = 5;
    constexpr std::size_t pull_frame_count = 30;
    constexpr std::size_t release_frame_count = 60;
    constexpr float delta_time = 1.0f / 60.0f;
    for (std::size_t i = 0; i < warmup_count; ++i)
    {
        if (!solver.step(delta_time))
        {
            std::cerr << "Warmup step failed.\n";
            return 1;
        }
    }

    const clock_type::time_point pull_begin = clock_type::now();
    for (std::size_t i = 0; i < pull_frame_count; ++i)
    {
        if (!solver.step(delta_time, pull_accelerations))
        {
            std::cerr << "Pull simulation step failed.\n";
            return 1;
        }
    }
    const clock_type::time_point pull_end = clock_type::now();
    const std::vector<boneflex::vec3> positions_after_pull = capture_positions(solver.particles());

    const clock_type::time_point release_begin = clock_type::now();
    for (std::size_t i = 0; i < release_frame_count; ++i)
    {
        if (!solver.step(delta_time))
        {
            std::cerr << "Release simulation step failed.\n";
            return 1;
        }
    }
    const clock_type::time_point release_end = clock_type::now();
    const std::vector<boneflex::vec3> positions_after_release = capture_positions(solver.particles());

    std::vector<std::size_t> all_indices(mesh.control_point_count());
    for (std::size_t i = 0; i < all_indices.size(); ++i)
    {
        all_indices[i] = i;
    }

    const displacement_metrics pull_all = measure_displacement(
        mesh.rest_positions(), positions_after_pull, all_indices);
    const displacement_metrics release_all = measure_displacement(
        mesh.rest_positions(), positions_after_release, all_indices);
    const displacement_metrics pull_local = measure_displacement(
        mesh.rest_positions(), positions_after_pull, pull_region);
    const displacement_metrics pull_neighbor = measure_displacement(
        mesh.rest_positions(), positions_after_pull, neighbor_region);
    const displacement_metrics pull_far = measure_displacement(
        mesh.rest_positions(), positions_after_pull, far_region);
    const displacement_metrics release_local = measure_displacement(
        mesh.rest_positions(), positions_after_release, pull_region);
    const displacement_metrics release_neighbor = measure_displacement(
        mesh.rest_positions(), positions_after_release, neighbor_region);
    const displacement_metrics release_far = measure_displacement(
        mesh.rest_positions(), positions_after_release, far_region);

    const double pull_average_ms = elapsed_ms(pull_begin, pull_end) / pull_frame_count;
    const double release_average_ms = elapsed_ms(release_begin, release_end) / release_frame_count;
    const double total_average_ms = (elapsed_ms(pull_begin, pull_end)
        + elapsed_ms(release_begin, release_end))
        / static_cast<double>(pull_frame_count + release_frame_count);

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "BoneFlex localized-pull soft body benchmark\n";
    std::cout << "Control points:              " << solver.particle_count() << '\n';
    std::cout << "Shape matching clusters:     " << clusters.cluster_count() << '\n';
    std::cout << "XPBD distance constraints:   " << distance_constraints.size() << '\n';
    std::cout << "XPBD compliance:             " << constraint_options.compliance << '\n';
    std::cout << "Solver iterations:           " << options.solver_iterations << '\n';
    std::cout << "Pull acceleration:           12.0 m/s^2 (+X, smooth falloff)\n";
    std::cout << "Pull / release frames:       " << pull_frame_count << " / " << release_frame_count << '\n';
    std::cout << "Average pull frame time:     " << pull_average_ms << " ms\n";
    std::cout << "Average release frame time:  " << release_average_ms << " ms\n";
    std::cout << "Average total frame time:    " << total_average_ms << " ms\n";
    std::cout << "\nDisplacement after pull (from rest)\n";
    std::cout << "Overall mean / RMS / max:    " << pull_all.mean << " / "
        << pull_all.rms << " / " << pull_all.maximum << '\n';
    print_region("Pull region", pull_local);
    print_region("Neighbor region", pull_neighbor);
    print_region("Far region", pull_far);
    std::cout << "\nDisplacement after release (from rest)\n";
    std::cout << "Overall mean / RMS / max:    " << release_all.mean << " / "
        << release_all.rms << " / " << release_all.maximum << '\n';
    print_region("Pull region", release_local);
    print_region("Neighbor region", release_neighbor);
    print_region("Far region", release_far);
    return 0;
}
