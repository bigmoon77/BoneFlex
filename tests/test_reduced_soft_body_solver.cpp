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

    void test_anchored_shape_matching_restores_translation()
    {
        boneflex::reduced_mesh mesh = make_grid(3);
        std::vector<std::uint32_t> indices(mesh.control_point_count());
        for (std::size_t i = 0; i < indices.size(); ++i)
            indices[i] = static_cast<std::uint32_t>(i);
        boneflex::shape_matching_clusters clusters;
        const std::uint32_t cluster_end = static_cast<std::uint32_t>(indices.size());
        require(clusters.set_data(mesh.control_point_count(), {0, cluster_end},
            std::move(indices)), "Failed to create anchored shape matching cluster.");

        std::vector<boneflex::vec3> current_positions = mesh.rest_positions();
        const std::size_t anchor_index = 4;
        const boneflex::vec3 translation(0.0f, -2.0f, 0.0f);
        for (std::size_t i = 0; i < current_positions.size(); ++i)
        {
            if (i == anchor_index) continue;
            current_positions[i] += translation;
        }

        std::vector<float> fixed_point_weights(mesh.control_point_count(), 0.0f);
        fixed_point_weights[anchor_index] = 1.0f;
        std::vector<boneflex::vec3> matched_positions;
        require(boneflex::solve_shape_matching(
            mesh.rest_positions(), current_positions, clusters,
            fixed_point_weights, 1.0f, matched_positions),
            "Anchored shape matching failed.");

        for (std::size_t i = 0; i < matched_positions.size(); ++i)
        {
            require((matched_positions[i] - mesh.rest_positions()[i]).length() < 1.0e-4f,
                "Kinematic anchor did not prevent rigid cluster translation.");
        }
    }

    void test_shape_matching_remains_stable_without_external_forces()
    {
        boneflex::reduced_mesh mesh = make_grid(4);
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 8;
        cluster_options.overlap_ratio = 0.75f;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters),
            "Failed to create shape matching stability clusters.");

        std::vector<boneflex::particle> particles(mesh.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = mesh.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
        }
        particles[0].inverse_mass = 0.0f;
        particles.back().position.y += 0.25f;
        particles.back().previous_position = particles.back().position;

        boneflex::reduced_soft_body_options options;
        options.gravity = boneflex::vec3();
        options.shape_matching_stiffness = 0.15f;
        options.max_deformation_ratio = 0.0f;
        options.solver_iterations = 4;
        options.velocity_damping = 0.995f;

        boneflex::reduced_soft_body_solver solver;
        require(solver.initialize(particles, mesh.rest_positions(), {}, clusters, options),
            "Shape matching stability solver initialization failed.");

        for (int frame = 0; frame < 240; ++frame)
        {
            require(solver.step(1.0f / 60.0f), "Shape matching stability step failed.");
            const auto& simulated_particles = solver.particles();
            for (std::size_t i = 0; i < simulated_particles.size(); ++i)
            {
                const boneflex::particle& value = simulated_particles[i];
                require(std::isfinite(value.position.x) && std::isfinite(value.position.y)
                    && std::isfinite(value.position.z), "Shape matching produced a non-finite position.");
                require(std::isfinite(value.velocity.x) && std::isfinite(value.velocity.y)
                    && std::isfinite(value.velocity.z), "Shape matching produced a non-finite velocity.");
                require(value.velocity.length() < 20.0f,
                    "Shape matching generated an excessive velocity without external forces.");
                require((value.position - mesh.rest_positions()[i]).length() < 1.0f,
                    "Shape matching caused unbounded displacement without external forces.");
            }
        }
    }

    void test_drag_target_does_not_change_fixed_anchor_configuration()
    {
        boneflex::reduced_mesh mesh = make_grid(4);
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 8;
        cluster_options.overlap_ratio = 0.75f;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters),
            "Failed to create drag target clusters.");

        std::vector<boneflex::particle> particles(mesh.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = mesh.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
        }
        particles[0].inverse_mass = 0.0f;

        boneflex::reduced_soft_body_options options;
        options.gravity = boneflex::vec3();
        options.shape_matching_stiffness = 0.15f;
        options.max_deformation_ratio = 0.0f;
        options.solver_iterations = 4;

        boneflex::reduced_soft_body_solver solver;
        require(solver.initialize(particles, mesh.rest_positions(), {}, clusters, options),
            "Drag target solver initialization failed.");

        const std::size_t dragged_index = particles.size() - 1;
        const boneflex::vec3 original_position = solver.particles()[dragged_index].position;
        const boneflex::vec3 target_position = original_position + boneflex::vec3(0.0f, 0.25f, 0.0f);
        const std::vector<boneflex::xpbd_position_target> targets = {
            { dragged_index, target_position, 1.0e-4f }
        };

        require(solver.step(1.0f / 60.0f, {}, targets), "Drag target solver step failed.");
        const auto& result = solver.particles();
        require(result[0].inverse_mass == 0.0f,
            "A permanent kinematic anchor changed inverse mass.");
        require((result[0].position - particles[0].position).length() < 1.0e-7f,
            "A permanent kinematic anchor moved while dragging another particle.");
        require(result[dragged_index].inverse_mass == 1.0f,
            "A drag target changed the particle inverse mass.");
        require((result[dragged_index].position - original_position).length() > 1.0e-4f,
            "The XPBD drag target did not move its particle.");
        require((result[dragged_index].position - target_position).length()
            < (original_position - target_position).length(),
            "The XPBD drag target did not move toward its target.");
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
        require(solver.set_distance_constraint_compliance(1.0e-4f),
            "Runtime XPBD compliance update failed.");
        require(std::fabs(solver.distance_constraints()[0].compliance() - 1.0e-4f) < 1.0e-8f,
            "Runtime XPBD compliance was not applied.");
        require(solver.set_distance_constraint_compliance(0.0f),
            "XPBD compliance reset failed.");
        options.solver_iterations = 2;
        require(solver.set_options(options), "Runtime solver option update failed.");
        require(solver.options().solver_iterations == 2,
            "Runtime solver iteration count was not applied.");
        require(solver.step(1.0f / 60.0f), "Constraint solver step failed.");
        const float distance = (solver.particles()[0].position - solver.particles()[1].position).length();
        require(std::fabs(distance - 1.0f) < 1.0e-4f, "Integrated XPBD constraint was not satisfied.");
    }


    void test_maximum_deformation_ratio()
    {
        boneflex::reduced_mesh mesh = make_grid(2);
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 4;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters),
            "Failed to create deformation-limit clusters.");

        std::vector<boneflex::particle> particles(mesh.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = mesh.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
        }
        particles[0].position.x += 4.0f;
        particles[0].previous_position = particles[0].position;

        std::vector<boneflex::xpbd_distance_constraint> constraints;
        constraints.emplace_back(0, 1, 1.0f, 1.0e6f);
        constraints.emplace_back(0, 2, 1.0f, 1.0e6f);

        std::vector<boneflex::vec3> fitted_positions;
        std::vector<boneflex::vec3> current_positions;
        for (const boneflex::particle& value : particles)
            current_positions.push_back(value.position);
        require(boneflex::solve_shape_matching(
            mesh.rest_positions(), current_positions, clusters, 1.0f, fitted_positions),
            "Failed to calculate the fitted reference shape.");

        boneflex::reduced_soft_body_options options;
        options.gravity = boneflex::vec3();
        options.shape_matching_stiffness = 0.5f;
        options.max_deformation_ratio = 0.1f;
        options.solver_iterations = 1;

        boneflex::reduced_soft_body_solver solver;
        require(solver.initialize(particles, mesh.rest_positions(), constraints, clusters, options),
            "Deformation-limit solver initialization failed.");
        require(solver.step(1.0f / 60.0f), "Deformation-limit solver step failed.");

        const float residual = (solver.particles()[0].position - fitted_positions[0]).length();
        require(residual <= 0.1001f,
            "Maximum deformation ratio did not clamp displacement from the fitted shape.");
    }

    void test_per_particle_material_parameters()
    {
        boneflex::reduced_mesh mesh = make_grid(2);
        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 4;
        boneflex::shape_matching_clusters clusters;
        require(boneflex::generate_shape_matching_clusters(mesh, cluster_options, clusters), "Material cluster generation failed.");
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
        boneflex::reduced_soft_body_solver solver;
        require(solver.initialize(particles, mesh.rest_positions(), {}, clusters, options), "Material solver initialization failed.");
        std::vector<boneflex::reduced_soft_body_material_parameters> parameters(particles.size());
        for (auto& value : parameters) { value.shape_matching_stiffness = 0.0f; value.velocity_damping = 0.25f; value.max_deformation_ratio = 0.0f; }
        require(solver.set_material_parameters(parameters), "Material parameter update failed.");
        auto updated_options = solver.options();
        updated_options.gravity = boneflex::vec3(0.0f, -1.0f, 0.0f);
        require(solver.set_options(updated_options), "Unrelated runtime option update failed.");
        require(solver.step(1.0f / 60.0f), "Material solver step failed.");
        require(std::fabs(solver.particles()[0].velocity.x - 0.25f) < 1.0e-5f, "Material damping was not applied.");
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
        test_anchored_shape_matching_restores_translation();
        test_shape_matching_remains_stable_without_external_forces();
        test_drag_target_does_not_change_fixed_anchor_configuration();
        test_distance_constraint_is_solved();
        test_maximum_deformation_ratio();
        test_velocity_damping_is_frame_rate_scaled();
        test_per_particle_material_parameters();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "All reduced soft body solver tests passed.\n";
    return 0;
}
