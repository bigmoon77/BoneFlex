#include <boneflex/animation/skinning.h>

namespace boneflex
{
    bool skin_vertices(
        const std::vector<vec3>& bind_positions,
        const skinning_weights& weights,
        const std::vector<transform>& skinning_palette,
        std::vector<vec3>& output_positions
    )
    {
        if (bind_positions.size() != weights.vertex_count())
        {
            return false;
        }

        const std::vector<std::uint32_t>& offsets = weights.offsets();
        const std::vector<std::uint32_t>& bone_indices = weights.bone_indices();
        const std::vector<float>& influence_weights = weights.weights();

        std::vector<vec3> result(bind_positions.size());

        for (std::size_t vertex_index = 0; vertex_index < bind_positions.size(); ++vertex_index)
        {
            vec3 skinned_position;
            for (std::uint32_t influence_index = offsets[vertex_index];
                 influence_index < offsets[vertex_index + 1];
                 ++influence_index)
            {
                const std::size_t index = influence_index;
                const std::uint32_t bone_index = bone_indices[index];
                if (bone_index >= skinning_palette.size())
                {
                    return false;
                }

                skinned_position += skinning_palette[bone_index].transform_point(
                    bind_positions[vertex_index]
                ) * influence_weights[index];
            }

            result[vertex_index] = skinned_position;
        }

        output_positions = std::move(result);
        return true;
    }
}
