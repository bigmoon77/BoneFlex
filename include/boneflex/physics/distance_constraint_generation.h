#pragma once

#include <boneflex/physics/reduced_mesh.h>
#include <boneflex/physics/xpbd_distance_constraint.h>

#include <string>
#include <vector>

namespace boneflex
{
    struct distance_constraint_generation_options
    {
        float compliance = 0.0f;
        std::vector<float> material_compliances;
    };

    // Generates one XPBD distance constraint for each unique undirected
    // surface edge. Rest lengths are measured from the reduced mesh.
    bool generate_distance_constraints(
        const reduced_mesh& mesh,
        const distance_constraint_generation_options& options,
        std::vector<xpbd_distance_constraint>& output_constraints,
        std::string* error_message = nullptr
    );
}
