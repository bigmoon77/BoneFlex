#include <boneflex/physics/distance_constraint_generation.h>
#include <boneflex/physics/reduced_mesh.h>
#include <boneflex/physics/reduced_soft_body_solver.h>
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

    struct deformation_metrics
    {
        double mean = 0.0;
        double rms = 0.0;
        double maximum = 0.0;
    };

    double elapsed_ms(clock_type::time_point begin, clock_type::time_point end)
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
        constexpr float pi = 3.14159265358979323846f;
        positions.reserve(static_cast<std::size_t>(longitude_segments) * latitude_segments);

        for (std::uint32_t y = 0; y < latitude_segments; ++y)
        {
            const float latitude = pi * static_cast<float>(y)
                / static_cast<float>(latitude_segments - 1);
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const float longitude = 2.0f * pi * static_cast<float>(x)
                    / static_cast<float>(longitude_segments);
                positions.emplace_back(
                    std::sin(latitude) * std::cos(longitude),
                    std::cos(latitude),
                    std::sin(latitude) * std::sin(longitude));
            }
        }

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

    boneflex::vec3 geometric_center(const std::vector<boneflex::particle>& particles)
    {
        boneflex::vec3 center;
        for (const boneflex::particle& value : particles)
        {
            center += value.position;
        }
        if (!particles.empty())
        {
            center = center / static_cast<float>(particles.size());
        }
        return center;
    }

    deformation_metrics measure_deformation(
        const std::vector<boneflex::vec3>& rest_positions,
        const std::vector<boneflex::particle>& particles,
        const std::vector<std::size_t>& region)
    {
        deformation_metrics metrics;
        if (region.empty() || particles.size() != rest_positions.size())
        {
            return metrics;
        }

        const boneflex::vec3 rest_center = [&rest_positions]()
        {
            boneflex::vec3 center;
            for (const boneflex::vec3& position : rest_positions)
            {
                center += position;
            }
            return center / static_cast<float>(rest_positions.size());
        }();
        const boneflex::vec3 current_center = geometric_center(particles);

        double sum = 0.0;
        double squared_sum = 0.0;
        for (std::size_t index : region)
        {
            const boneflex::vec3 rest_local = rest_positions[index] - rest_center;
            const boneflex::vec3 current_local = particles[index].position - current_center;
            const boneflex::vec3 difference = current_local - rest_local;
            const double distance = std::sqrt(static_cast<double>(difference.length_squared()));
            sum += distance;
            squared_sum += distance * distance;
            metrics.maximum = std::max(metrics.maximum, distance);
        }

        metrics.mean = sum / static_cast<double>(region.size());
        metrics.rms = std::sqrt(squared_sum / static_cast<double>(region.size()));
        return metrics;
    }

    double measure_velocity_rms(
        const std::vector<boneflex::particle>& particles,
        const std::vector<std::size_t>& region)
    {
        if (region.empty())
        {
            return 0.0;
        }

        double squared_sum = 0.0;
        for (std::size_t index : region)
        {
            squared_sum += static_cast<double>(particles[index].velocity.length_squared());
        }
        return std::sqrt(squared_sum / static_cast<double>(region.size()));
    }

    bool run_case(
        const boneflex::reduced_mesh& mesh,
        const std::vector<boneflex::xpbd_distance_constraint>& distance_constraints,
        std::size_t cluster_size,
        float velocity_damping,
        const std::vector<boneflex::vec3>& pull_accelerations,
        const std::vector<std::size_t>& pull_region,
        const std::vector<std::size_t>& far_region)
    {
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = cluster_size;
        cluster_options.overlap_ratio = 0.5f;

        boneflex::shape_matching_clusters clusters;
        std::string error_message;
        const clock_type::time_point cluster_begin = clock_type::now();
        if (!boneflex::generate_shape_matching_clusters(
            mesh, cluster_options, clusters, &error_message))
        {
            std::cerr << "Cluster generation failed: " << error_message << '\n';
            return false;
        }
        const clock_type::time_point cluster_end = clock_type::now();

        std::vector<boneflex::particle> particles(mesh.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = mesh.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
        }

        boneflex::reduced_soft_body_options solver_options;
        solver_options.solver_iterations = 4;
        constexpr float stiffness = 0.80f;
        solver_options.shape_matching_stiffness = stiffness;
        solver_options.gravity = boneflex::vec3();
        solver_options.velocity_damping = velocity_damping;

        boneflex::reduced_soft_body_solver solver;
        if (!solver.initialize(
            particles, mesh.rest_positions(), distance_constraints, clusters, solver_options))
        {
            std::cerr << "Solver initialization failed.\n";
            return false;
        }

        constexpr std::size_t warmup_count = 5;
        constexpr std::size_t pull_frame_count = 30;
        constexpr std::size_t release_sample_interval = 15;
        constexpr std::size_t release_frame_count = 120;
        constexpr float delta_time = 1.0f / 60.0f;

        for (std::size_t i = 0; i < warmup_count; ++i)
        {
            if (!solver.step(delta_time))
            {
                std::cerr << "Warmup step failed.\n";
                return false;
            }
        }

        const clock_type::time_point pull_begin = clock_type::now();
        for (std::size_t i = 0; i < pull_frame_count; ++i)
        {
            if (!solver.step(delta_time, pull_accelerations))
            {
                std::cerr << "Pull step failed.\n";
                return false;
            }
        }
        const clock_type::time_point pull_end = clock_type::now();
        const deformation_metrics pull_metrics =
            measure_deformation(mesh.rest_positions(), solver.particles(), pull_region);
        const double pull_time = elapsed_ms(pull_begin, pull_end)
            / static_cast<double>(pull_frame_count);

        const deformation_metrics initial_far_metrics =
            measure_deformation(mesh.rest_positions(), solver.particles(), far_region);
        std::cout << std::setw(7) << cluster_size
            << std::setw(11) << velocity_damping
            << std::setw(11) << stiffness
            << std::setw(11) << clusters.cluster_count()
            << std::setw(13) << clusters.control_point_indices().size()
            << std::setw(13) << pull_metrics.rms
            << std::setw(12) << 0
            << std::setw(14) << pull_metrics.rms
            << std::setw(13) << initial_far_metrics.rms
            << std::setw(13) << 0.0
            << std::setw(13) << measure_velocity_rms(solver.particles(), pull_region)
            << std::setw(13) << pull_time
            << std::setw(14) << 0.0
            << std::setw(13) << elapsed_ms(cluster_begin, cluster_end)
            << '\n';

        std::size_t released_frames = 0;
        while (released_frames < release_frame_count)
        {
            const clock_type::time_point interval_begin = clock_type::now();
            for (std::size_t i = 0; i < release_sample_interval; ++i)
            {
                if (!solver.step(delta_time))
                {
                    std::cerr << "Release step failed.\n";
                    return false;
                }
            }
            const clock_type::time_point interval_end = clock_type::now();
            released_frames += release_sample_interval;

            const deformation_metrics release_metrics =
                measure_deformation(mesh.rest_positions(), solver.particles(), pull_region);
            const deformation_metrics far_metrics =
                measure_deformation(mesh.rest_positions(), solver.particles(), far_region);
            const double recovery_percent = pull_metrics.rms > 1.0e-8
                ? (1.0 - release_metrics.rms / pull_metrics.rms) * 100.0
                : 0.0;
            const double release_time = elapsed_ms(interval_begin, interval_end)
                / static_cast<double>(release_sample_interval);

            const double velocity_rms =
                measure_velocity_rms(solver.particles(), pull_region);

            std::cout << std::setw(7) << cluster_size
                << std::setw(11) << velocity_damping
                << std::setw(11) << stiffness
                << std::setw(11) << clusters.cluster_count()
                << std::setw(13) << clusters.control_point_indices().size()
                << std::setw(13) << pull_metrics.rms
                << std::setw(12) << released_frames
                << std::setw(14) << release_metrics.rms
                << std::setw(13) << far_metrics.rms
                << std::setw(13) << recovery_percent
                << std::setw(13) << velocity_rms
                << std::setw(13) << pull_time
                << std::setw(14) << release_time
                << std::setw(13) << elapsed_ms(cluster_begin, cluster_end)
                << '\n';
        }
        return true;
    }
}

int main(int argc, char** argv)
{
    std::uint32_t longitude_segments = 200;
    std::uint32_t latitude_segments = 50;
    if (argc > 1)
    {
        longitude_segments = static_cast<std::uint32_t>(std::stoul(argv[1]));
    }
    if (argc > 2)
    {
        latitude_segments = static_cast<std::uint32_t>(std::stoul(argv[2]));
    }
    if (longitude_segments < 3 || latitude_segments < 3)
    {
        std::cerr << "Usage: boneflex_reduced_soft_body_parameter_sweep [longitude_segments] [latitude_segments]\n";
        return 1;
    }

    boneflex::reduced_mesh mesh;
    if (!build_uv_sphere(longitude_segments, latitude_segments, mesh))
    {
        std::cerr << "Failed to build benchmark sphere.\n";
        return 1;
    }

    boneflex::distance_constraint_generation_options constraint_options;
    constraint_options.compliance = 1.0e-6f;
    std::vector<boneflex::xpbd_distance_constraint> distance_constraints;
    std::string error_message;
    if (!boneflex::generate_distance_constraints(
        mesh, constraint_options, distance_constraints, &error_message))
    {
        std::cerr << "Distance constraint generation failed: " << error_message << '\n';
        return 1;
    }

    std::vector<boneflex::vec3> pull_accelerations(mesh.control_point_count());
    std::vector<std::size_t> pull_region;
    std::vector<std::size_t> far_region;
    for (std::size_t i = 0; i < mesh.control_point_count(); ++i)
    {
        const float x = mesh.rest_positions()[i].x;
        const float falloff = std::clamp((x - 0.65f) / 0.35f, 0.0f, 1.0f);
        const float smooth_falloff = falloff * falloff * (3.0f - 2.0f * falloff);
        pull_accelerations[i] = boneflex::vec3(12.0f * smooth_falloff, 0.0f, 0.0f);
        if (x >= 0.90f)
        {
            pull_region.push_back(i);
        }
        if (x <= 0.0f)
        {
            far_region.push_back(i);
        }
    }

    constexpr std::size_t cluster_sizes[] = {128};
    constexpr float damping_values[] = {0.93f, 0.94f, 0.95f, 0.96f, 0.97f};

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "BoneFlex reduced soft-body parameter sweep\n";
    std::cout << "Control points: " << mesh.control_point_count()
        << ", triangles: " << mesh.triangle_count()
        << ", XPBD constraints: " << distance_constraints.size() << '\n';
    std::cout << "Fixed settings: overlap=0.50, iterations=4, compliance=1.0e-6, stiffness=0.80\n";
    std::cout << "Sweep: cluster size 128, velocity damping 0.93/0.94/0.95/0.96/0.97\n";
    std::cout << "Pull/release: 30/120 frames at 60 Hz; release sampled every 15 frames\n";
    std::cout << "Metrics remove geometric-center translation; recovery is relative to pull RMS\n";
    std::cout << "\n"
        << std::setw(7) << "Cluster"
        << std::setw(11) << "Damping"
        << std::setw(11) << "Stiffness"
        << std::setw(11) << "Clusters"
        << std::setw(13) << "Memberships"
        << std::setw(13) << "Pull RMS"
        << std::setw(12) << "Release fr"
        << std::setw(14) << "Release RMS"
        << std::setw(13) << "Far RMS"
        << std::setw(13) << "Recovery %"
        << std::setw(13) << "Velocity RMS"
        << std::setw(13) << "Pull ms/f"
        << std::setw(14) << "Release ms/f"
        << std::setw(13) << "Build ms"
        << '\n';

    for (std::size_t cluster_size : cluster_sizes)
    {
        for (float damping : damping_values)
        {
            if (!run_case(
                mesh, distance_constraints, cluster_size, damping,
                pull_accelerations, pull_region, far_region))
            {
                return 1;
            }
        }
    }
    return 0;
}
