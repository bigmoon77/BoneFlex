#include <boneflex/physics/distance_constraint_generation.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    boneflex::reduced_mesh make_mesh()
    {
        boneflex::reduced_mesh mesh;
        require(mesh.set_data(
            {
                boneflex::vec3(0.0f, 0.0f, 0.0f),
                boneflex::vec3(1.0f, 0.0f, 0.0f),
                boneflex::vec3(0.0f, 1.0f, 0.0f),
                boneflex::vec3(1.0f, 1.0f, 0.0f)
            },
            {0, 1, 2, 1, 3, 2}),
            "Failed to create test mesh.");
        return mesh;
    }

    void test_unique_edges_and_rest_lengths()
    {
        const boneflex::reduced_mesh mesh = make_mesh();
        boneflex::distance_constraint_generation_options options;
        options.compliance = 0.001f;
        std::vector<boneflex::xpbd_distance_constraint> constraints;
        std::string error_message;
        require(boneflex::generate_distance_constraints(mesh, options, constraints, &error_message),
            "Constraint generation failed.");
        require(error_message.empty(), "Successful generation returned an error.");
        require(constraints.size() == 5, "Shared triangle edge was not deduplicated.");

        bool found_diagonal = false;
        for (const boneflex::xpbd_distance_constraint& constraint : constraints)
        {
            require(constraint.particle_a() < constraint.particle_b(),
                "Generated edge indices are not canonical.");
            require(std::fabs(constraint.compliance() - options.compliance) < 1.0e-7f,
                "Generated constraint compliance is incorrect.");
            const boneflex::vec3 difference =
                mesh.rest_positions()[constraint.particle_a()]
                - mesh.rest_positions()[constraint.particle_b()];
            require(std::fabs(constraint.rest_length() - difference.length()) < 1.0e-6f,
                "Generated constraint rest length is incorrect.");
            if (constraint.particle_a() == 1 && constraint.particle_b() == 2)
            {
                found_diagonal = true;
                require(std::fabs(constraint.rest_length() - std::sqrt(2.0f)) < 1.0e-6f,
                    "Shared diagonal rest length is incorrect.");
            }
        }
        require(found_diagonal, "Shared diagonal constraint is missing.");
    }

    void test_material_compliance_is_applied_per_triangle()
    {
        boneflex::reduced_mesh mesh = make_mesh();
        require(mesh.set_triangle_material_indices({0, 1}), "Failed to set triangle materials.");
        boneflex::distance_constraint_generation_options options;
        options.material_compliances = {0.001f, 0.003f};
        std::vector<boneflex::xpbd_distance_constraint> constraints;
        require(boneflex::generate_distance_constraints(mesh, options, constraints), "Material-aware constraint generation failed.");
        for (const auto& constraint : constraints)
        {
            const float expected = (constraint.particle_a() == 1 && constraint.particle_b() == 2)
                ? 0.002f
                : ((constraint.particle_a() == 0 || constraint.particle_b() == 0) ? 0.001f : 0.003f);
            require(std::fabs(constraint.compliance() - expected) < 1.0e-6f, "Material compliance was not applied.");
        }
    }

    void test_invalid_options_preserve_output()
    {
        const boneflex::reduced_mesh mesh = make_mesh();
        std::vector<boneflex::xpbd_distance_constraint> constraints;
        constraints.emplace_back(0, 1, 3.0f);
        boneflex::distance_constraint_generation_options options;
        options.compliance = -1.0f;
        std::string error_message;
        require(!boneflex::generate_distance_constraints(mesh, options, constraints, &error_message),
            "Negative compliance was accepted.");
        require(!error_message.empty(), "Invalid input did not report an error.");
        require(constraints.size() == 1 && constraints[0].rest_length() == 3.0f,
            "Output was modified after generation failed.");
    }
}

int main()
{
    try
    {
        test_unique_edges_and_rest_lengths();
        test_material_compliance_is_applied_per_triangle();
        test_invalid_options_preserve_output();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "All distance constraint generation tests passed.\n";
    return 0;
}
