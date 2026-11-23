#include <boneflex/physics/xpbd_distance_constraint.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void test_distance_constraint_corrects_stretched_particles()
    {
        std::vector<boneflex::particle> particles(2);
        particles[0].position = boneflex::vec3(0.0f, 0.0f, 0.0f);
        particles[1].position = boneflex::vec3(2.0f, 0.0f, 0.0f);

        boneflex::xpbd_distance_constraint constraint(0, 1, 1.0f);
        constraint.solve(particles, 1.0f / 60.0f);

        const float distance = (particles[0].position - particles[1].position).length();
        require(std::fabs(distance - 1.0f) < 1.0e-5f, "Stretched distance was not corrected.");
    }

    void test_fixed_particle_remains_stationary()
    {
        std::vector<boneflex::particle> particles(2);
        particles[0].position = boneflex::vec3(0.0f, 0.0f, 0.0f);
        particles[0].inverse_mass = 0.0f;
        particles[1].position = boneflex::vec3(2.0f, 0.0f, 0.0f);

        boneflex::xpbd_distance_constraint constraint(0, 1, 1.0f);
        constraint.solve(particles, 1.0f / 60.0f);

        require(particles[0].position.length() < 1.0e-6f, "Fixed particle moved.");
        require(std::fabs((particles[0].position - particles[1].position).length() - 1.0f) < 1.0e-5f,
            "Movable particle did not satisfy the constraint.");
    }

    void test_compliant_constraint_remains_soft()
    {
        std::vector<boneflex::particle> particles(2);
        particles[0].position = boneflex::vec3(0.0f, 0.0f, 0.0f);
        particles[1].position = boneflex::vec3(2.0f, 0.0f, 0.0f);

        boneflex::xpbd_distance_constraint constraint(0, 1, 1.0f, 1.0e-3f);
        constraint.solve(particles, 1.0f / 60.0f);

        const float distance = (particles[0].position - particles[1].position).length();
        require(distance > 1.0f && distance < 2.0f, "Compliant constraint did not produce a soft correction.");
    }
}

int main()
{
    try
    {
        test_distance_constraint_corrects_stretched_particles();
        test_fixed_particle_remains_stationary();
        test_compliant_constraint_remains_soft();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }

    std::cout << "All BoneFlex tests passed.\n";
    return 0;
}
