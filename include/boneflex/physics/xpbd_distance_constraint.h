#pragma once

#include <boneflex/physics/particle.h>

#include <cstddef>
#include <vector>

namespace boneflex
{
    class xpbd_distance_constraint
    {
    public:
        xpbd_distance_constraint(
            std::size_t particle_a,
            std::size_t particle_b,
            float rest_length,
            float compliance = 0.0f
        );

        void reset_lambda();
        void solve(std::vector<particle>& particles, float delta_time);

        std::size_t particle_a() const;
        std::size_t particle_b() const;
        float rest_length() const;
        float compliance() const;
        bool set_compliance(float compliance);
        float lambda() const;

    private:
        std::size_t _particle_a = 0;
        std::size_t _particle_b = 0;
        float _rest_length = 0.0f;
        float _compliance = 0.0f;
        float _lambda = 0.0f;
    };
}
