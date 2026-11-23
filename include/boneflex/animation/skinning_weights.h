#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace boneflex
{
    // Compressed sparse row (CSR) layout: vertex i uses
    // [offsets[i], offsets[i + 1]) in bone_indices and weights.
    class skinning_weights
    {
    public:
        bool set_data(
            std::size_t vertex_count,
            std::vector<std::uint32_t> offsets,
            std::vector<std::uint32_t> bone_indices,
            std::vector<float> weights,
            std::size_t bone_count,
            std::string* error_message = nullptr
        );

        std::size_t vertex_count() const;
        std::size_t influence_count() const;
        const std::vector<std::uint32_t>& offsets() const;
        const std::vector<std::uint32_t>& bone_indices() const;
        const std::vector<float>& weights() const;

    private:
        std::size_t _vertex_count = 0;
        std::vector<std::uint32_t> _offsets;
        std::vector<std::uint32_t> _bone_indices;
        std::vector<float> _weights;
    };
}
