#include <boneflex/physics/reduced_soft_body_solver.h>
#include <boneflex/physics/reduced_mesh.h>
#include <boneflex/physics/shape_matching_cluster_generation.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    boneflex::reduced_mesh make_grid(std::uint32_t side)
    {
        std::vector<boneflex::vec3> positions;
        std::vector<std::uint32_t> indices;
        for (std::uint32_t y = 0; y < side; ++y)
            for (std::uint32_t x = 0; x < side; ++x)
                positions.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
        for (std::uint32_t y = 0; y + 1 < side; ++y)
            for (std::uint32_t x = 0; x + 1 < side; ++x)
            {
                const std::uint32_t a = y * side + x;
                const std::uint32_t b = a + 1;
                const std::uint32_t c = a + side;
                const std::uint32_t d = c + 1;
                indices.insert(indices.end(), {a, b, c, b, d, c});
            }
        boneflex::reduced_mesh mesh;
        require(mesh.set_data(std::move(positions), std::move(indices)), "Failed to create test grid.");
        return mesh;
    }

    void test_gravity_and_fixed_particle()
    {
        boneflex::reduced_mesh mesh = make_grid(4);
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 8;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters),
            "Failed to create shape matching clusters.");

        std::vector<boneflex::particle> particles(mesh.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = mesh.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
        }
        particles[0].inverse_mass = 0.0f;

        boneflex::reduced_soft_body_options options;
        options.gravity = boneflex::vec3(0.0f, -10.0f, 0.0f);
        options.solver_iterations = 3;
        options.shape_matching_stiffness = 0.1f;
        boneflex::reduced_soft_body_solver solver;
        require(solver.initialize(particles, mesh.rest_positions(), {}, clusters, options),
            "Solver initialization failed.");
        require(solver.step(1.0f / 60.0f), "Solver step failed.");

        const std::vector<boneflex::particle>& result = solver.particles();
        require((result[0].position - particles[0].position).length() < 1.0e-7f,
            "Fixed particle moved.");
        require(result[5].position.y < particles[5].position.y,
            "Gravity did not move a dynamic particle.");
    }

    void test_distance_constraint_is_solved()
    {
        std::vector<boneflex::vec3> rest_positions = {
            boneflex::vec3(0.0f, 0.0f, 0.0f),
            boneflex::vec3(1.0f, 0.0f, 0.0f),
            boneflex::vec3(0.0f, 1.0f, 0.0f),
            boneflex::vec3(1.0f, 1.0f, 0.0f)
        };
        boneflex::reduced_mesh mesh;
        require(mesh.set_data(rest_positions, {0, 1, 2, 1, 3, 2}), "Failed to create constraint mesh.");
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 4;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters),
            "Failed to create constraint clusters.");

        std::vector<boneflex::particle> particles(4);
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = rest_positions[i];
            particles[i].previous_position = rest_positions[i];
        }
        particles[1].position.x = 1.5f;
        particles[1].previous_position = particles[1].position;

        std::vector<boneflex::xpbd_distance_constraint> constraints;
        constraints.emplace_back(0, 1, 1.0f, 0.0f);
        boneflex::reduced_soft_body_options options;
        options.gravity = boneflex::vec3();
        options.shape_matching_stiffness = 0.0f;
        options.solver_iterations = 4;
        boneflex::reduced_soft_body_solver solver;
        require(solver.initialize(particles, rest_positions, constraints, clusters, options),
            "Constraint solver initialization failed.");
        require(solver.step(1.0f / 60.0f), "Constraint solver step failed.");
        const float distance = (solver.particles()[0].position - solver.particles()[1].position).length();
        require(std::fabs(distance - 1.0f) < 1.0e-4f, "Integrated XPBD constraint was not satisfied.");
    }

    void test_velocity_damping_is_frame_rate_scaled()
    {
        boneflex::reduced_mesh mesh = make_grid(2);
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 4;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters),
            "Failed to create damping test clusters.");

        std::vector<boneflex::particle> particles(mesh.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = mesh.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
            particles[i].velocity = boneflex::vec3(1.0f, 0.0f, 0.0f);
        }

        boneflex::reduced_soft_body_options options;
        options.gravity = boneflex::vec3();
        options.shape_matching_stiffness = 0.0f;
        options.velocity_damping = 0.5f;

        boneflex::reduced_soft_body_solver solver_60_hz;
        require(solver_60_hz.initialize(particles, mesh.rest_positions(), {}, clusters, options),
            "60 Hz damping solver initialization failed.");
        require(solver_60_hz.step(1.0f / 60.0f), "60 Hz damping solver step failed.");
        require(std::fabs(solver_60_hz.particles()[0].velocity.x - 0.5f) < 1.0e-5f,
            "Damping at 60 Hz did not apply one frame of velocity retention.");

        boneflex::reduced_soft_body_solver solver_30_hz;
        require(solver_30_hz.initialize(particles, mesh.rest_positions(), {}, clusters, options),
            "30 Hz damping solver initialization failed.");
        require(solver_30_hz.step(1.0f / 30.0f), "30 Hz damping solver step failed.");
        require(std::fabs(solver_30_hz.particles()[0].velocity.x - 0.25f) < 1.0e-5f,
            "Damping at 30 Hz was not scaled to two 60 Hz frames.");
    }
}

int main()
{
    try
    {
        test_gravity_and_fixed_particle();
        test_distance_constraint_is_solved();
        test_velocity_damping_is_frame_rate_scaled();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "All reduced soft body solver tests passed.\n";
    return 0;
}
