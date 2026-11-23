#pragma once

#include <boneflex/physics/particle.h>
#include <boneflex/physics/shape_matching.h>
#include <boneflex/physics/xpbd_distance_constraint.h>

#include <cstddef>
#include <vector>

namespace boneflex
{
    struct reduced_soft_body_options
    {
        std::size_t solver_iterations = 4;
        float shape_matching_stiffness = 0.15f;
        // Maximum residual from the locally fitted shape, as a multiple of the average incident rest-edge length. Zero disables the limit.
        float max_deformation_ratio = 0.5f;
        vec3 gravity = vec3(0.0f, -9.81f, 0.0f);
        // Velocity retention per 1/60-second frame.
        float velocity_damping = 0.995f;
    };

    // CPU reference solver combining local XPBD distance constraints with
    // overlapping clustered shape matching on reduced control points.
    class reduced_soft_body_solver
    {
    public:
        bool initialize(
            const std::vector<particle>& particles,
            const std::vector<vec3>& rest_positions,
            const std::vector<xpbd_distance_constraint>& distance_constraints,
            const shape_matching_clusters& clusters,
            const reduced_soft_body_options& options = {}
        );

        bool step(float delta_time, const std::vector<vec3>& external_accelerations = {});

        std::size_t particle_count() const;
        const std::vector<particle>& particles() const;
        std::vector<particle>& particles();
        const std::vector<vec3>& rest_positions() const;
        const std::vector<xpbd_distance_constraint>& distance_constraints() const;
        const reduced_soft_body_options& options() const;

    private:
        std::vector<particle> _particles;
        std::vector<vec3> _rest_positions;
        std::vector<float> _max_deformation_distances;
        std::vector<xpbd_distance_constraint> _distance_constraints;
        shape_matching_clusters _clusters;
        reduced_soft_body_options _options;
        bool _initialized = false;
    };
}
