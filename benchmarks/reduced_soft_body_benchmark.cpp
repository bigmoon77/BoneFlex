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

    boneflex::vec3 calculate_geometric_center(const std::vector<boneflex::vec3>& positions)
    {
        boneflex::vec3 center;
        if (positions.empty())
        {
            return center;
        }
        for (const boneflex::vec3& position : positions)
        {
            center += position;
        }
        return center / static_cast<float>(positions.size());
    }

    struct quaternion
    {
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    boneflex::vec3 rotate_vector(const quaternion& rotation, const boneflex::vec3& value)
    {
        const boneflex::vec3 q_vector(
            static_cast<float>(rotation.x),
            static_cast<float>(rotation.y),
            static_cast<float>(rotation.z));
        const boneflex::vec3 twice_cross(
            2.0f * (q_vector.y * value.z - q_vector.z * value.y),
            2.0f * (q_vector.z * value.x - q_vector.x * value.z),
            2.0f * (q_vector.x * value.y - q_vector.y * value.x));
        const boneflex::vec3 cross_again(
            q_vector.y * twice_cross.z - q_vector.z * twice_cross.y,
            q_vector.z * twice_cross.x - q_vector.x * twice_cross.z,
            q_vector.x * twice_cross.y - q_vector.y * twice_cross.x);
        return value + static_cast<float>(rotation.w) * twice_cross + cross_again;
    }

    quaternion conjugate(const quaternion& value)
    {
        return quaternion{value.w, -value.x, -value.y, -value.z};
    }

    quaternion largest_eigenvector_symmetric_4x4(double matrix[4][4])
    {
        double eigenvectors[4][4]{};
        for (std::size_t i = 0; i < 4; ++i)
        {
            eigenvectors[i][i] = 1.0;
        }

        // Jacobi diagonalization keeps this small eigenproblem dependency-free.
        for (std::size_t iteration = 0; iteration < 64; ++iteration)
        {
            std::size_t p = 0;
            std::size_t q = 1;
            double largest_off_diagonal = 0.0;
            for (std::size_t row = 0; row < 4; ++row)
            {
                for (std::size_t column = row + 1; column < 4; ++column)
                {
                    const double magnitude = std::abs(matrix[row][column]);
                    if (magnitude > largest_off_diagonal)
                    {
                        largest_off_diagonal = magnitude;
                        p = row;
                        q = column;
                    }
                }
            }
            if (largest_off_diagonal < 1.0e-12)
            {
                break;
            }

            const double angle = 0.5 * std::atan2(
                2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            const double pp = matrix[p][p];
            const double qq = matrix[q][q];
            const double pq = matrix[p][q];

            for (std::size_t index = 0; index < 4; ++index)
            {
                if (index == p || index == q)
                {
                    continue;
                }
                const double ip = matrix[index][p];
                const double iq = matrix[index][q];
                matrix[index][p] = cosine * ip - sine * iq;
                matrix[p][index] = matrix[index][p];
                matrix[index][q] = sine * ip + cosine * iq;
                matrix[q][index] = matrix[index][q];
            }

            matrix[p][p] = cosine * cosine * pp
                - 2.0 * sine * cosine * pq + sine * sine * qq;
            matrix[q][q] = sine * sine * pp
                + 2.0 * sine * cosine * pq + cosine * cosine * qq;
            matrix[p][q] = 0.0;
            matrix[q][p] = 0.0;

            for (std::size_t row = 0; row < 4; ++row)
            {
                const double vp = eigenvectors[row][p];
                const double vq = eigenvectors[row][q];
                eigenvectors[row][p] = cosine * vp - sine * vq;
                eigenvectors[row][q] = sine * vp + cosine * vq;
            }
        }

        std::size_t largest_index = 0;
        for (std::size_t i = 1; i < 4; ++i)
        {
            if (matrix[i][i] > matrix[largest_index][largest_index])
            {
                largest_index = i;
            }
        }

        quaternion result{
            eigenvectors[0][largest_index],
            eigenvectors[1][largest_index],
            eigenvectors[2][largest_index],
            eigenvectors[3][largest_index]
        };
        const double length = std::sqrt(
            result.w * result.w + result.x * result.x
            + result.y * result.y + result.z * result.z);
        if (length > 0.0)
        {
            result.w /= length;
            result.x /= length;
            result.y /= length;
            result.z /= length;
        }
        return result;
    }

    std::vector<boneflex::vec3> align_rigid_transform_to_rest(
        const std::vector<boneflex::vec3>& rest_positions,
        const std::vector<boneflex::vec3>& current_positions)
    {
        std::vector<boneflex::vec3> aligned_positions;
        if (rest_positions.size() != current_positions.size() || rest_positions.empty())
        {
            return aligned_positions;
        }

        const boneflex::vec3 rest_center = calculate_geometric_center(rest_positions);
        const boneflex::vec3 current_center = calculate_geometric_center(current_positions);
        double covariance[3][3]{};
        for (std::size_t i = 0; i < rest_positions.size(); ++i)
        {
            const boneflex::vec3 rest = rest_positions[i] - rest_center;
            const boneflex::vec3 current = current_positions[i] - current_center;
            const double rest_values[3] = {rest.x, rest.y, rest.z};
            const double current_values[3] = {current.x, current.y, current.z};
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    covariance[row][column] += rest_values[row] * current_values[column];
                }
            }
        }

        const double sxx = covariance[0][0];
        const double sxy = covariance[0][1];
        const double sxz = covariance[0][2];
        const double syx = covariance[1][0];
        const double syy = covariance[1][1];
        const double syz = covariance[1][2];
        const double szx = covariance[2][0];
        const double szy = covariance[2][1];
        const double szz = covariance[2][2];

        // Horn's absolute orientation method: largest eigenvector is the
        // unit quaternion mapping the rest pose onto the current pose.
        double horn_matrix[4][4] = {
            {sxx + syy + szz, syz - szy, szx - sxz, sxy - syx},
            {syz - szy, sxx - syy - szz, sxy + syx, szx + sxz},
            {szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy},
            {sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz}
        };
        const quaternion rest_to_current = largest_eigenvector_symmetric_4x4(horn_matrix);
        const quaternion current_to_rest = conjugate(rest_to_current);

        aligned_positions.reserve(current_positions.size());
        for (const boneflex::vec3& position : current_positions)
        {
            aligned_positions.push_back(
                rotate_vector(current_to_rest, position - current_center) + rest_center);
        }
        return aligned_positions;
    }

    bool validate_rigid_alignment()
    {
        const std::vector<boneflex::vec3> rest_positions = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 2.0f, 0.0f},
            {0.0f, 0.0f, 3.0f},
            {-1.0f, -1.0f, -1.0f},
            {0.3f, 0.8f, -0.2f}
        };
        const double half_angle = 0.35;
        const double axis_length = std::sqrt(14.0);
        const quaternion rotation{
            std::cos(half_angle),
            std::sin(half_angle) / axis_length,
            2.0 * std::sin(half_angle) / axis_length,
            3.0 * std::sin(half_angle) / axis_length
        };
        const boneflex::vec3 translation(2.0f, -3.0f, 4.0f);
        std::vector<boneflex::vec3> transformed_positions;
        transformed_positions.reserve(rest_positions.size());
        for (const boneflex::vec3& position : rest_positions)
        {
            transformed_positions.push_back(rotate_vector(rotation, position) + translation);
        }

        const std::vector<boneflex::vec3> aligned_positions =
            align_rigid_transform_to_rest(rest_positions, transformed_positions);
        if (aligned_positions.size() != rest_positions.size())
        {
            return false;
        }
        for (std::size_t i = 0; i < rest_positions.size(); ++i)
        {
            if ((aligned_positions[i] - rest_positions[i]).length_squared() > 1.0e-8f)
            {
                return false;
            }
        }
        return true;
    }

    displacement_metrics measure_deformation(
        const std::vector<boneflex::vec3>& rest_positions,
        const std::vector<boneflex::vec3>& current_positions,
        const std::vector<std::size_t>& indices,
        const boneflex::vec3& rest_center,
        const boneflex::vec3& current_center)
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
            const boneflex::vec3 rest_local = rest_positions[index] - rest_center;
            const boneflex::vec3 current_local = current_positions[index] - current_center;
            const boneflex::vec3 deformation = current_local - rest_local;
            const double magnitude = std::sqrt(static_cast<double>(deformation.length_squared()));
            sum += magnitude;
            squared_sum += magnitude * magnitude;
            pull_axis_sum += deformation.x;
            metrics.maximum = std::max(metrics.maximum, magnitude);
        }
        metrics.mean = sum / static_cast<double>(metrics.count);
        metrics.rms = std::sqrt(squared_sum / static_cast<double>(metrics.count));
        metrics.mean_pull_axis = pull_axis_sum / static_cast<double>(metrics.count);
        return metrics;
    }

    void print_vector(const char* name, const boneflex::vec3& value)
    {
        std::cout << name << value.x << ", " << value.y << ", " << value.z
            << " (length: " << value.length() << ")\n";
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
    if (!validate_rigid_alignment())
    {
        std::cerr << "Rigid alignment validation failed.\n";
        return 1;
    }

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

    const boneflex::vec3 rest_center = calculate_geometric_center(mesh.rest_positions());
    const boneflex::vec3 pull_center = calculate_geometric_center(positions_after_pull);
    const boneflex::vec3 release_center = calculate_geometric_center(positions_after_release);

    const std::vector<boneflex::vec3> pull_rigid_aligned =
        align_rigid_transform_to_rest(mesh.rest_positions(), positions_after_pull);
    const std::vector<boneflex::vec3> release_rigid_aligned =
        align_rigid_transform_to_rest(mesh.rest_positions(), positions_after_release);

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

    const displacement_metrics pull_deformation_all = measure_deformation(
        mesh.rest_positions(), positions_after_pull, all_indices, rest_center, pull_center);
    const displacement_metrics release_deformation_all = measure_deformation(
        mesh.rest_positions(), positions_after_release, all_indices, rest_center, release_center);
    const displacement_metrics pull_deformation_local = measure_deformation(
        mesh.rest_positions(), positions_after_pull, pull_region, rest_center, pull_center);
    const displacement_metrics pull_deformation_neighbor = measure_deformation(
        mesh.rest_positions(), positions_after_pull, neighbor_region, rest_center, pull_center);
    const displacement_metrics pull_deformation_far = measure_deformation(
        mesh.rest_positions(), positions_after_pull, far_region, rest_center, pull_center);
    const displacement_metrics release_deformation_local = measure_deformation(
        mesh.rest_positions(), positions_after_release, pull_region, rest_center, release_center);
    const displacement_metrics release_deformation_neighbor = measure_deformation(
        mesh.rest_positions(), positions_after_release, neighbor_region, rest_center, release_center);
    const displacement_metrics release_deformation_far = measure_deformation(
        mesh.rest_positions(), positions_after_release, far_region, rest_center, release_center);

    const displacement_metrics pull_rigid_all = measure_displacement(
        mesh.rest_positions(), pull_rigid_aligned, all_indices);
    const displacement_metrics release_rigid_all = measure_displacement(
        mesh.rest_positions(), release_rigid_aligned, all_indices);
    const displacement_metrics pull_rigid_local = measure_displacement(
        mesh.rest_positions(), pull_rigid_aligned, pull_region);
    const displacement_metrics pull_rigid_neighbor = measure_displacement(
        mesh.rest_positions(), pull_rigid_aligned, neighbor_region);
    const displacement_metrics pull_rigid_far = measure_displacement(
        mesh.rest_positions(), pull_rigid_aligned, far_region);
    const displacement_metrics release_rigid_local = measure_displacement(
        mesh.rest_positions(), release_rigid_aligned, pull_region);
    const displacement_metrics release_rigid_neighbor = measure_displacement(
        mesh.rest_positions(), release_rigid_aligned, neighbor_region);
    const displacement_metrics release_rigid_far = measure_displacement(
        mesh.rest_positions(), release_rigid_aligned, far_region);

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

    std::cout << "\nCenter of mass translation (geometric center)\n";
    print_vector("Rest center:    ", rest_center);
    print_vector("After pull:     ", pull_center);
    print_vector("Pull offset:    ", pull_center - rest_center);
    print_vector("After release:  ", release_center);
    print_vector("Release offset: ", release_center - rest_center);

    std::cout << "\nDeformation after pull (center translation removed)\n";
    std::cout << "Overall mean / RMS / max:    " << pull_deformation_all.mean << " / "
        << pull_deformation_all.rms << " / " << pull_deformation_all.maximum << '\n';
    print_region("Pull region", pull_deformation_local);
    print_region("Neighbor region", pull_deformation_neighbor);
    print_region("Far region", pull_deformation_far);

    std::cout << "\nDeformation after release (center translation removed)\n";
    std::cout << "Overall mean / RMS / max:    " << release_deformation_all.mean << " / "
        << release_deformation_all.rms << " / " << release_deformation_all.maximum << '\n';
    print_region("Pull region", release_deformation_local);
    print_region("Neighbor region", release_deformation_neighbor);
    print_region("Far region", release_deformation_far);

    std::cout << "\nDeformation after pull (best-fit rigid transform removed)\n";
    std::cout << "Overall mean / RMS / max:    " << pull_rigid_all.mean << " / "
        << pull_rigid_all.rms << " / " << pull_rigid_all.maximum << '\n';
    print_region("Pull region", pull_rigid_local);
    print_region("Neighbor region", pull_rigid_neighbor);
    print_region("Far region", pull_rigid_far);

    std::cout << "\nDeformation after release (best-fit rigid transform removed)\n";
    std::cout << "Overall mean / RMS / max:    " << release_rigid_all.mean << " / "
        << release_rigid_all.rms << " / " << release_rigid_all.maximum << '\n';
    print_region("Pull region", release_rigid_local);
    print_region("Neighbor region", release_rigid_neighbor);
    print_region("Far region", release_rigid_far);
    return 0;
}
