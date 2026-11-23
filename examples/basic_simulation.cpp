#include <boneflex/physics/xpbd_distance_constraint.h>

#include <iostream>
#include <vector>

int main()
{
    std::vector<boneflex::particle> particles(2);
    particles[0].position = boneflex::vec3(0.0f, 0.0f, 0.0f);
    particles[1].position = boneflex::vec3(2.0f, 0.0f, 0.0f);

    boneflex::xpbd_distance_constraint constraint(0, 1, 1.0f);
    constraint.solve(particles, 1.0f / 60.0f);

    const float final_distance = (particles[0].position - particles[1].position).length();
    std::cout << "Distance after one XPBD solve: " << final_distance << '\n';
    return 0;
}
