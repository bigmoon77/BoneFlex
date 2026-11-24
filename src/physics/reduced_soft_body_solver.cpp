#include <boneflex/physics/reduced_soft_body_solver.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace boneflex
{
    namespace
    {
        bool finite_vec3(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool valid_options(const reduced_soft_body_options& options)
        {
            return options.solver_iterations > 0
                && std::isfinite(options.shape_matching_stiffness)
                && options.shape_matching_stiffness >= 0.0f
                && options.shape_matching_stiffness <= 1.0f
                && std::isfinite(options.max_deformation_ratio)
                && options.max_deformation_ratio >= 0.0f
                && finite_vec3(options.gravity)
                && std::isfinite(options.velocity_damping)
                && options.velocity_damping >= 0.0f
                && options.velocity_damping <= 1.0f;
        }
    }

    bool reduced_soft_body_solver::initialize(
        const std::vector<particle>& particles,
        const std::vector<vec3>& rest_positions,
        const std::vector<xpbd_distance_constraint>& distance_constraints,
        const shape_matching_clusters& clusters,
        const reduced_soft_body_options& options)
    {
        if (particles.empty() || particles.size() != rest_positions.size()
            || clusters.control_point_count() != particles.size()
            || clusters.cluster_count() == 0 || !valid_options(options))
        {
            return false;
        }

        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            const particle& value = particles[i];
            if (!finite_vec3(value.position) || !finite_vec3(value.previous_position)
                || !finite_vec3(value.velocity) || !finite_vec3(rest_positions[i])
                || !std::isfinite(value.inverse_mass) || value.inverse_mass < 0.0f)
            {
                return false;
            }
        }
        for (const xpbd_distance_constraint& constraint : distance_constraints)
        {
            if (constraint.particle_a() >= particles.size()
                || constraint.particle_b() >= particles.size())
            {
                return false;
            }
        }

        _particles = particles;
        _rest_positions = rest_positions;
        _distance_constraints = distance_constraints;
        _max_deformation_distances.assign(particles.size(), 0.0f);
        _shape_matching_stiffnesses.assign(particles.size(), options.shape_matching_stiffness);
        _velocity_dampings.assign(particles.size(), options.velocity_damping);
        _max_deformation_ratios.assign(particles.size(), options.max_deformation_ratio);
        std::vector<float> incident_edge_length_sums(particles.size(), 0.0f);
        std::vector<std::size_t> incident_edge_counts(particles.size(), 0);
        for (const xpbd_distance_constraint& constraint : _distance_constraints)
        {
            const std::size_t a = constraint.particle_a();
            const std::size_t b = constraint.particle_b();
            incident_edge_length_sums[a] += constraint.rest_length();
            incident_edge_length_sums[b] += constraint.rest_length();
            ++incident_edge_counts[a];
            ++incident_edge_counts[b];
        }
        if (options.max_deformation_ratio > 0.0f)
        {
            for (std::size_t i = 0; i < particles.size(); ++i)
            {
                if (incident_edge_counts[i] == 0) continue;
                const float average_edge_length = incident_edge_length_sums[i]
                    / static_cast<float>(incident_edge_counts[i]);
                _max_deformation_distances[i] = average_edge_length * options.max_deformation_ratio;
            }
        }
        _clusters = clusters;
        _options = options;
        _initialized = true;
        return true;
    }

    bool reduced_soft_body_solver::step(
        float delta_time,
        const std::vector<vec3>& external_accelerations,
        const std::vector<xpbd_position_target>& position_targets)
    {
        if (!_initialized || !std::isfinite(delta_time) || delta_time <= 0.0f
            || (!external_accelerations.empty() && external_accelerations.size() != _particles.size()))
        {
            return false;
        }
        for (const vec3& acceleration : external_accelerations)
        {
            if (!finite_vec3(acceleration))
            {
                return false;
            }
        }
        for (const xpbd_position_target& target : position_targets)
        {
            if (target.particle_index >= _particles.size()
                || !finite_vec3(target.position)
                || !std::isfinite(target.compliance)
                || target.compliance < 0.0f)
            {
                return false;
            }
        }

        const float delta_time_squared = delta_time * delta_time;
        if (!std::isfinite(delta_time_squared) || delta_time_squared <= 0.0f)
        {
            return false;
        }

        for (particle& value : _particles)
        {
            value.previous_position = value.position;
            if (value.inverse_mass <= 0.0f)
            {
                value.velocity = vec3();
                continue;
            }

            vec3 acceleration = _options.gravity;
            if (!external_accelerations.empty())
            {
                acceleration += external_accelerations[&value - _particles.data()];
            }
            value.velocity += acceleration * delta_time;
            value.position += value.velocity * delta_time;
        }

        for (xpbd_distance_constraint& constraint : _distance_constraints)
        {
            constraint.reset_lambda();
        }

        std::vector<vec3> current_positions(_particles.size());
        std::vector<float> fixed_point_weights(_particles.size(), 0.0f);
        for (std::size_t i = 0; i < _particles.size(); ++i)
            fixed_point_weights[i] = _particles[i].inverse_mass <= 0.0f ? 1.0f : 0.0f;
        std::vector<vec3> matched_positions;
        // XPBD multipliers are accumulated across solver iterations and reset each frame.
        std::vector<vec3> position_target_lambdas(position_targets.size());
        std::vector<float> iteration_stiffnesses(_particles.size(), 0.0f);
        for (std::size_t i = 0; i < _particles.size(); ++i)
        {
            const float stiffness = _shape_matching_stiffnesses[i];
            iteration_stiffnesses[i] = stiffness <= 0.0f ? 0.0f
                : 1.0f - std::pow(1.0f - stiffness, 1.0f / static_cast<float>(_options.solver_iterations));
        }
        for (std::size_t iteration = 0; iteration < _options.solver_iterations; ++iteration)
        {
            for (xpbd_distance_constraint& constraint : _distance_constraints)
            {
                constraint.solve(_particles, delta_time);
            }

            if (std::any_of(iteration_stiffnesses.begin(), iteration_stiffnesses.end(), [](float value) { return value > 0.0f; }))
            {
                for (std::size_t i = 0; i < _particles.size(); ++i)
                {
                    current_positions[i] = _particles[i].position;
                }
                if (!solve_shape_matching(
                    _rest_positions, current_positions, _clusters,
                    fixed_point_weights, 1.0f, matched_positions))
                {
                    return false;
                }
                for (std::size_t i = 0; i < _particles.size(); ++i)
                {
                    if (_particles[i].inverse_mass <= 0.0f) continue;

                    const vec3 current = _particles[i].position;
                    const vec3 fitted = matched_positions[i];
                    vec3 deformation = current + (fitted - current)
                        * iteration_stiffnesses[i] - fitted;

                    const float maximum_deformation = _max_deformation_distances[i];
                    if (maximum_deformation > 0.0f)
                    {
                        const float deformation_length = deformation.length();
                        if (deformation_length > maximum_deformation)
                        {
                            deformation *= maximum_deformation / deformation_length;
                        }
                    }
                    _particles[i].position = fitted + deformation;
                }
            }

            // Apply temporary interaction targets after shape matching. The target
            // remains a soft XPBD constraint and never becomes a permanent anchor.
            for (std::size_t i = 0; i < position_targets.size(); ++i)
            {
                const xpbd_position_target& target = position_targets[i];
                particle& value = _particles[target.particle_index];
                if (value.inverse_mass <= 0.0f) continue;

                const float alpha = target.compliance / delta_time_squared;
                const float denominator = value.inverse_mass + alpha;
                if (denominator <= 0.0f) continue;

                const vec3 constraint_value = value.position - target.position;
                const vec3 delta_lambda = (
                    constraint_value * -1.0f - position_target_lambdas[i] * alpha)
                    / denominator;
                value.position += delta_lambda * value.inverse_mass;
                position_target_lambdas[i] += delta_lambda;
            }
        }

        // velocity_damping is the velocity retention per 1/60-second frame.
        // Scale the exponent so the damping remains consistent at other time steps.
        for (std::size_t i = 0; i < _particles.size(); ++i)
        {
            particle& value = _particles[i];
            if (value.inverse_mass > 0.0f)
            {
                value.velocity = (value.position - value.previous_position) / delta_time;
                value.velocity *= std::pow(_velocity_dampings[i], delta_time * 60.0f);
            }
            else
            {
                value.velocity = vec3();
            }
        }
        return true;
    }

    std::size_t reduced_soft_body_solver::particle_count() const { return _particles.size(); }
    const std::vector<particle>& reduced_soft_body_solver::particles() const { return _particles; }
    std::vector<particle>& reduced_soft_body_solver::particles() { return _particles; }
    const std::vector<vec3>& reduced_soft_body_solver::rest_positions() const { return _rest_positions; }
    const std::vector<xpbd_distance_constraint>& reduced_soft_body_solver::distance_constraints() const { return _distance_constraints; }
    const reduced_soft_body_options& reduced_soft_body_solver::options() const { return _options; }

    bool reduced_soft_body_solver::set_options(const reduced_soft_body_options& options)
    {
        if (!_initialized || !valid_options(options))
            return false;

        const bool stiffness_changed = options.shape_matching_stiffness != _options.shape_matching_stiffness;
        const bool damping_changed = options.velocity_damping != _options.velocity_damping;
        const bool deformation_changed = options.max_deformation_ratio != _options.max_deformation_ratio;
        _options = options;
        if (stiffness_changed)
            std::fill(_shape_matching_stiffnesses.begin(), _shape_matching_stiffnesses.end(), options.shape_matching_stiffness);
        if (damping_changed)
            std::fill(_velocity_dampings.begin(), _velocity_dampings.end(), options.velocity_damping);
        if (deformation_changed)
        {
            std::fill(_max_deformation_ratios.begin(), _max_deformation_ratios.end(), options.max_deformation_ratio);
            std::vector<float> edge_length_sums(_particles.size(), 0.0f);
            std::vector<std::size_t> edge_counts(_particles.size(), 0);
            for (const xpbd_distance_constraint& constraint : _distance_constraints)
            {
                const std::size_t a = constraint.particle_a(), b = constraint.particle_b();
                edge_length_sums[a] += constraint.rest_length();
                edge_length_sums[b] += constraint.rest_length();
                ++edge_counts[a]; ++edge_counts[b];
            }
            for (std::size_t i = 0; i < _particles.size(); ++i)
                _max_deformation_distances[i] = edge_counts[i] == 0 ? 0.0f
                    : edge_length_sums[i] / static_cast<float>(edge_counts[i]) * _max_deformation_ratios[i];
        }
        return true;
    }

    bool reduced_soft_body_solver::set_material_parameters(const std::vector<reduced_soft_body_material_parameters>& parameters)
    {
        if (!_initialized || parameters.size() != _particles.size()) return false;
        for (const auto& value : parameters)
            if (!std::isfinite(value.shape_matching_stiffness) || value.shape_matching_stiffness < 0.0f || value.shape_matching_stiffness > 1.0f
                || !std::isfinite(value.velocity_damping) || value.velocity_damping < 0.0f || value.velocity_damping > 1.0f
                || !std::isfinite(value.max_deformation_ratio) || value.max_deformation_ratio < 0.0f) return false;
        std::vector<float> edge_sums(_particles.size(), 0.0f);
        std::vector<std::size_t> edge_counts(_particles.size(), 0);
        for (const auto& constraint : _distance_constraints)
        {
            const auto a = constraint.particle_a(), b = constraint.particle_b();
            edge_sums[a] += constraint.rest_length(); edge_sums[b] += constraint.rest_length();
            ++edge_counts[a]; ++edge_counts[b];
        }
        for (std::size_t i = 0; i < parameters.size(); ++i)
        {
            _shape_matching_stiffnesses[i] = parameters[i].shape_matching_stiffness;
            _velocity_dampings[i] = parameters[i].velocity_damping;
            _max_deformation_ratios[i] = parameters[i].max_deformation_ratio;
            _max_deformation_distances[i] = edge_counts[i] == 0 ? 0.0f
                : edge_sums[i] / static_cast<float>(edge_counts[i]) * parameters[i].max_deformation_ratio;
        }
        return true;
    }

    bool reduced_soft_body_solver::set_distance_constraint_compliance(float compliance)
    {
        if (!_initialized || !std::isfinite(compliance) || compliance < 0.0f)
            return false;
        for (xpbd_distance_constraint& constraint : _distance_constraints)
            constraint.set_compliance(compliance);
        return true;
    }
}
