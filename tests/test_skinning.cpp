#include <boneflex/animation/skinning.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace
{
    bool nearly_equal(float left, float right, float epsilon = 1.0e-4f)
    {
        return std::abs(left - right) <= epsilon;
    }

    bool test_single_bone_skinning()
    {
        boneflex::skinning_weights weights;
        if (!weights.set_data(
            1,
            std::vector<std::uint32_t>{0, 1},
            std::vector<std::uint32_t>{0},
            std::vector<float>{1.0f},
            1))
        {
            return false;
        }

        std::vector<boneflex::transform> palette(1);
        palette[0].position = boneflex::vec3(2.0f, 3.0f, 4.0f);

        std::vector<boneflex::vec3> output;
        const bool success = boneflex::skin_vertices(
            std::vector<boneflex::vec3>{boneflex::vec3(1.0f, 0.0f, 0.0f)},
            weights,
            palette,
            output
        );

        return success && output.size() == 1
            && nearly_equal(output[0].x, 3.0f)
            && nearly_equal(output[0].y, 3.0f)
            && nearly_equal(output[0].z, 4.0f);
    }

    bool test_weight_normalization()
    {
        boneflex::skinning_weights weights;
        if (!weights.set_data(
            1,
            std::vector<std::uint32_t>{0, 2},
            std::vector<std::uint32_t>{0, 1},
            std::vector<float>{2.0f, 2.0f},
            2))
        {
            return false;
        }

        return nearly_equal(weights.weights()[0], 0.5f)
            && nearly_equal(weights.weights()[1], 0.5f);
    }

    bool test_rejects_invalid_bone_index()
    {
        boneflex::skinning_weights weights;
        std::string error_message;
        return !weights.set_data(
            1,
            std::vector<std::uint32_t>{0, 1},
            std::vector<std::uint32_t>{2},
            std::vector<float>{1.0f},
            2,
            &error_message
        ) && !error_message.empty()
            && weights.vertex_count() == 0;
    }

    bool test_rejects_zero_weight_sum()
    {
        boneflex::skinning_weights weights;
        return !weights.set_data(
            1,
            std::vector<std::uint32_t>{0, 2},
            std::vector<std::uint32_t>{0, 1},
            std::vector<float>{0.0f, 0.0f},
            2
        );
    }
}

int main()
{
    if (!test_single_bone_skinning())
    {
        std::cerr << "Single bone skinning test failed.\n";
        return 1;
    }

    if (!test_weight_normalization())
    {
        std::cerr << "Weight normalization test failed.\n";
        return 1;
    }

    if (!test_rejects_invalid_bone_index())
    {
        std::cerr << "Invalid bone index test failed.\n";
        return 1;
    }

    if (!test_rejects_zero_weight_sum())
    {
        std::cerr << "Zero weight sum test failed.\n";
        return 1;
    }

    std::cout << "All skinning tests passed.\n";
    return 0;
}
