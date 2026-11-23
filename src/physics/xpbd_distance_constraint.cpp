#include <boneflex/physics/xpbd_distance_constraint.h>

#include <cmath>
#include <stdexcept>

namespace boneflex
{
    xpbd_distance_constraint::xpbd_distance_constraint(
        std::size_t particle_a,
        std::size_t particle_b,
        float rest_length,
        float compliance
    )
        : _particle_a(particle_a)
        , _particle_b(particle_b)
        , _rest_length(rest_length)
        , _compliance(compliance)
    {
        if (particle_a == particle_b)
        {
            throw std::invalid_argument("An XPBD distance constraint requires two different particles.");
        }
        if (!std::isfinite(rest_length) || rest_length < 0.0f)
        {
            throw std::invalid_argument("Rest length must be finite and non-negative.");
        }
        if (!std::isfinite(compliance) || compliance < 0.0f)
        {
            throw std::invalid_argument("Compliance must be finite and non-negative.");
        }
    }

    void xpbd_distance_constraint::reset_lambda()
    {
        _lambda = 0.0f;
    }

    void xpbd_distance_constraint::solve(std::vector<particle>& particles, float delta_time)
    {
        if (_particle_a >= particles.size() || _particle_b >= particles.size())
        {
            throw std::out_of_range("XPBD distance constraint particle index is out of range.");
        }
        if (!std::isfinite(delta_time) || delta_time <= 0.0f)
        {
            throw std::invalid_argument("Delta time must be finite and positive.");
        }

        particle& particle_a_value = particles[_particle_a];
        particle& particle_b_value = particles[_particle_b];

        const vec3 difference = particle_a_value.position - particle_b_value.position;
        const float distance = difference.length();
        if (distance <= 1.0e-8f)
        {
            return;
        }

        const float inverse_mass_sum =
            particle_a_value.inverse_mass + particle_b_value.inverse_mass;
        if (inverse_mass_sum <= 0.0f)
        {
            return;
        }

        const float alpha = _compliance / (delta_time * delta_time);
        const float constraint_value = distance - _rest_length;
        const float delta_lambda =
            (-constraint_value - alpha * _lambda) / (inverse_mass_sum + alpha);

        const vec3 gradient = difference / distance;
        particle_a_value.position += gradient * (particle_a_value.inverse_mass * delta_lambda);
        particle_b_value.position -= gradient * (particle_b_value.inverse_mass * delta_lambda);
        _lambda += delta_lambda;
    }

    std::size_t xpbd_distance_constraint::particle_a() const
    {
        return _particle_a;
    }

    std::size_t xpbd_distance_constraint::particle_b() const
    {
        return _particle_b;
    }

    float xpbd_distance_constraint::rest_length() const
    {
        return _rest_length;
    }

    float xpbd_distance_constraint::compliance() const
    {
        return _compliance;
    }

    float xpbd_distance_constraint::lambda() const
    {
        return _lambda;
    }
}
