#include <boneflex/animation/skinning_weights.h>

#include <cmath>
#include <utility>

namespace boneflex
{
    bool skinning_weights::set_data(
        std::size_t vertex_count,
        std::vector<std::uint32_t> offsets,
        std::vector<std::uint32_t> bone_indices,
        std::vector<float> weights,
        std::size_t bone_count,
        std::string* error_message
    )
    {
        const auto fail = [error_message](const char* message) -> bool
        {
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        };

        if (offsets.size() != vertex_count + 1)
        {
            return fail("Offsets must contain vertex_count + 1 elements.");
        }

        if (offsets.empty() || offsets.front() != 0)
        {
            return fail("The first offset must be zero.");
        }

        if (bone_indices.size() != weights.size())
        {
            return fail("Bone index and weight arrays must have equal sizes.");
        }

        if (offsets.back() != bone_indices.size())
        {
            return fail("The last offset must equal the influence count.");
        }

        for (std::size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            if (offsets[vertex_index] > offsets[vertex_index + 1])
            {
                return fail("Offsets must be monotonically non-decreasing.");
            }

            if (offsets[vertex_index] == offsets[vertex_index + 1])
            {
                return fail("Every vertex must have at least one bone influence.");
            }

            float weight_sum = 0.0f;
            for (std::uint32_t influence_index = offsets[vertex_index];
                 influence_index < offsets[vertex_index + 1];
                 ++influence_index)
            {
                const std::size_t index = influence_index;
                if (bone_indices[index] >= bone_count)
                {
                    return fail("A bone index is outside the skeleton.");
                }

                if (!std::isfinite(weights[index]) || weights[index] < 0.0f)
                {
                    return fail("Weights must be finite and non-negative.");
                }

                weight_sum += weights[index];
            }

            if (!std::isfinite(weight_sum) || weight_sum <= 1.0e-8f)
            {
                return fail("Each vertex must have a positive total weight.");
            }

            for (std::uint32_t influence_index = offsets[vertex_index];
                 influence_index < offsets[vertex_index + 1];
                 ++influence_index)
            {
                weights[influence_index] /= weight_sum;
            }
        }

        _vertex_count = vertex_count;
        _offsets = std::move(offsets);
        _bone_indices = std::move(bone_indices);
        _weights = std::move(weights);

        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }

    std::size_t skinning_weights::vertex_count() const
    {
        return _vertex_count;
    }

    std::size_t skinning_weights::influence_count() const
    {
        return _weights.size();
    }

    const std::vector<std::uint32_t>& skinning_weights::offsets() const
    {
        return _offsets;
    }

    const std::vector<std::uint32_t>& skinning_weights::bone_indices() const
    {
        return _bone_indices;
    }

    const std::vector<float>& skinning_weights::weights() const
    {
        return _weights;
    }
}
