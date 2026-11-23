#include <boneflex/physics/reduced_mesh.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    bool nearly_equal(float left, float right, float epsilon = 1.0e-4f)
    {
        return std::abs(left - right) <= epsilon;
    }

    bool test_reduced_mesh_validation()
    {
        boneflex::reduced_mesh mesh;
        if (!mesh.set_data(
            std::vector<boneflex::vec3>{
                boneflex::vec3(0.0f, 0.0f, 0.0f),
                boneflex::vec3(1.0f, 0.0f, 0.0f),
                boneflex::vec3(0.0f, 1.0f, 0.0f)
            },
            std::vector<std::uint32_t>{0, 1, 2}))
        {
            return false;
        }

        if (mesh.control_point_count() != 3 || mesh.triangle_count() != 1)
        {
            return false;
        }

        std::string error_message;
        return !mesh.set_data(
            std::vector<boneflex::vec3>{boneflex::vec3()},
            std::vector<std::uint32_t>{0, 1, 2},
            &error_message)
            && !error_message.empty()
            && mesh.control_point_count() == 0;
    }

    bool test_vertex_reconstruction()
    {
        boneflex::vertex_embedding embedding;
        if (!embedding.set_data(
            2,
            std::vector<std::uint32_t>{0, 2, 3},
            std::vector<std::uint32_t>{0, 1, 1},
            std::vector<float>{1.0f, 1.0f, 1.0f},
            2))
        {
            return false;
        }

        const std::vector<boneflex::vec3> control_positions{
            boneflex::vec3(0.0f, 0.0f, 0.0f),
            boneflex::vec3(2.0f, 4.0f, 6.0f)
        };
        std::vector<boneflex::vec3> output;
        if (!boneflex::reconstruct_render_vertices(control_positions, embedding, output))
        {
            return false;
        }

        return output.size() == 2
            && nearly_equal(output[0].x, 1.0f)
            && nearly_equal(output[0].y, 2.0f)
            && nearly_equal(output[0].z, 3.0f)
            && nearly_equal(output[1].x, 2.0f)
            && nearly_equal(output[1].y, 4.0f)
            && nearly_equal(output[1].z, 6.0f)
            && nearly_equal(embedding.weights()[0], 0.5f)
            && nearly_equal(embedding.weights()[1], 0.5f);
    }

    bool test_rejects_invalid_embedding()
    {
        boneflex::vertex_embedding embedding;
        std::string error_message;
        return !embedding.set_data(
            1,
            std::vector<std::uint32_t>{0, 1},
            std::vector<std::uint32_t>{3},
            std::vector<float>{1.0f},
            2,
            &error_message)
            && !error_message.empty()
            && embedding.render_vertex_count() == 0;
    }

    bool test_output_is_unchanged_on_failure()
    {
        boneflex::vertex_embedding embedding;
        if (!embedding.set_data(
            1,
            std::vector<std::uint32_t>{0, 1},
            std::vector<std::uint32_t>{0},
            std::vector<float>{1.0f},
            1))
        {
            return false;
        }

        std::vector<boneflex::vec3> output{boneflex::vec3(9.0f, 8.0f, 7.0f)};
        const bool success = boneflex::reconstruct_render_vertices(
            std::vector<boneflex::vec3>{},
            embedding,
            output
        );
        return !success && output.size() == 1 && nearly_equal(output[0].x, 9.0f);
    }
}

int main()
{
    if (!test_reduced_mesh_validation())
    {
        std::cerr << "Reduced mesh validation test failed.\n";
        return 1;
    }
    if (!test_vertex_reconstruction())
    {
        std::cerr << "Vertex reconstruction test failed.\n";
        return 1;
    }
    if (!test_rejects_invalid_embedding())
    {
        std::cerr << "Invalid embedding test failed.\n";
        return 1;
    }
    if (!test_output_is_unchanged_on_failure())
    {
        std::cerr << "Output preservation test failed.\n";
        return 1;
    }

    std::cout << "All reduced mesh tests passed.\n";
    return 0;
}
