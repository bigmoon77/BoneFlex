#include <boneflex/io/blender_asset.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace boneflex
{
    namespace
    {
        constexpr std::array<char, 4> asset_magic = { 'B', 'F', 'L', 'X' };
        constexpr std::uint32_t asset_version = 2;
        constexpr std::uint32_t min_asset_version = 1;
        constexpr std::uint32_t max_material_count = 65'535;
        constexpr std::uint32_t max_vertex_count = 50'000'000;
        constexpr std::uint32_t max_index_count = 300'000'000;
        constexpr std::uint32_t max_bone_count = 65'535;
        constexpr std::uint32_t max_influence_count = 500'000'000;
        constexpr std::uint32_t max_bone_name_length = 1'048'576;

        template<typename value_type>
        bool read_value(std::istream& stream, value_type& value)
        {
            static_assert(std::is_trivially_copyable_v<value_type>);
            stream.read(reinterpret_cast<char*>(&value), sizeof(value));
            return static_cast<bool>(stream);
        }

        bool read_values(std::istream& stream, void* destination, std::size_t byte_count)
        {
            if (byte_count == 0)
            {
                return true;
            }
            stream.read(static_cast<char*>(destination), static_cast<std::streamsize>(byte_count));
            return static_cast<bool>(stream);
        }

        bool fail(std::string* error_message, const char* message)
        {
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        }

        bool valid_count(std::uint32_t count, std::uint32_t maximum)
        {
            return count <= maximum;
        }
    }

    bool blender_asset::skin_current_pose(std::vector<vec3>& output_positions) const
    {
        const std::vector<transform> palette = rig.build_skinning_palette();
        return skin_vertices(bind_positions, weights, palette, output_positions);
    }

    bool load_blender_asset(
        const std::string& file_path,
        blender_asset& output_asset,
        std::string* error_message
    )
    {
        std::ifstream stream(file_path, std::ios::binary);
        if (!stream)
        {
            return fail(error_message, "Could not open the .bflex asset file.");
        }

        std::array<char, 4> magic{};
        if (!read_values(stream, magic.data(), magic.size()) || magic != asset_magic)
        {
            return fail(error_message, "Invalid .bflex file signature.");
        }

        std::uint32_t version = 0;
        std::uint32_t vertex_count = 0;
        std::uint32_t index_count = 0;
        std::uint32_t bone_count = 0;
        std::uint32_t influence_count = 0;
        std::uint32_t material_count = 0;
        if (!read_value(stream, version)
            || !read_value(stream, vertex_count)
            || !read_value(stream, index_count)
            || !read_value(stream, bone_count)
            || !read_value(stream, influence_count))
        {
            return fail(error_message, "The .bflex file header is truncated.");
        }

        if (version < min_asset_version || version > asset_version)
        {
            return fail(error_message, "Unsupported .bflex asset version.");
        }
        if (version >= 2 && !read_value(stream, material_count))
        {
            return fail(error_message, "The .bflex material count is truncated.");
        }
        if (version == 1) material_count = 1;
        if (material_count == 0 || !valid_count(material_count, max_material_count))
        {
            return fail(error_message, "The .bflex material count is invalid.");
        }
        if (vertex_count == 0 || !valid_count(vertex_count, max_vertex_count))
        {
            return fail(error_message, "The .bflex vertex count is invalid.");
        }
        if (index_count == 0 || index_count % 3 != 0 || !valid_count(index_count, max_index_count))
        {
            return fail(error_message, "The .bflex triangle index count is invalid.");
        }
        if (bone_count == 0 || !valid_count(bone_count, max_bone_count))
        {
            return fail(error_message, "The .bflex bone count is invalid.");
        }
        if (influence_count < vertex_count || !valid_count(influence_count, max_influence_count))
        {
            return fail(error_message, "The .bflex skinning influence count is invalid.");
        }

        blender_asset asset;
        asset.bind_positions.resize(vertex_count);
        for (vec3& position : asset.bind_positions)
        {
            if (!read_value(stream, position.x)
                || !read_value(stream, position.y)
                || !read_value(stream, position.z))
            {
                return fail(error_message, "The .bflex vertex position data is truncated.");
            }
        }

        asset.triangle_indices.resize(index_count);
        if (!read_values(stream, asset.triangle_indices.data(),
            static_cast<std::size_t>(index_count) * sizeof(std::uint32_t)))
        {
            return fail(error_message, "The .bflex triangle index data is truncated.");
        }
        for (const std::uint32_t index : asset.triangle_indices)
        {
            if (index >= vertex_count)
            {
                return fail(error_message, "A .bflex triangle index is outside the vertex array.");
            }
        }

        asset.triangle_material_indices.assign(index_count / 3, 0);
        if (version >= 2)
        {
            if (!read_values(stream, asset.triangle_material_indices.data(),
                asset.triangle_material_indices.size() * sizeof(std::uint32_t)))
            {
                return fail(error_message, "The .bflex triangle material data is truncated.");
            }
            for (const std::uint32_t material_index : asset.triangle_material_indices)
            {
                if (material_index >= material_count)
                {
                    return fail(error_message, "A triangle references an invalid material index.");
                }
            }

            asset.materials.reserve(material_count);
            for (std::uint32_t material_index = 0; material_index < material_count; ++material_index)
            {
                std::uint32_t name_length = 0;
                if (!read_value(stream, name_length) || name_length == 0 || name_length > max_bone_name_length)
                {
                    return fail(error_message, "A .bflex material name length is invalid.");
                }
                blender_material_physics material;
                material.name.resize(name_length);
                if (!read_values(stream, material.name.data(), name_length)
                    || !read_value(stream, material.shape_matching_stiffness)
                    || !read_value(stream, material.xpbd_compliance)
                    || !read_value(stream, material.velocity_damping)
                    || !read_value(stream, material.max_deformation_ratio))
                {
                    return fail(error_message, "The .bflex material data is truncated.");
                }
                if (!std::isfinite(material.shape_matching_stiffness)
                    || material.shape_matching_stiffness < 0.0f || material.shape_matching_stiffness > 1.0f
                    || !std::isfinite(material.xpbd_compliance) || material.xpbd_compliance < 0.0f
                    || !std::isfinite(material.velocity_damping)
                    || material.velocity_damping < 0.0f || material.velocity_damping > 1.0f
                    || !std::isfinite(material.max_deformation_ratio) || material.max_deformation_ratio < 0.0f)
                {
                    return fail(error_message, "A .bflex material contains invalid physics parameters.");
                }
                asset.materials.push_back(std::move(material));
            }
        }
        else
        {
            asset.materials.push_back(blender_material_physics{ "Default" });
        }

        std::vector<bone> bones;
        bones.reserve(bone_count);
        for (std::uint32_t bone_index = 0; bone_index < bone_count; ++bone_index)
        {
            std::uint32_t name_length = 0;
            if (!read_value(stream, name_length) || name_length == 0 || name_length > max_bone_name_length)
            {
                return fail(error_message, "A .bflex bone name length is invalid.");
            }

            bone value;
            value.name.resize(name_length);
            if (!read_values(stream, value.name.data(), name_length)
                || !read_value(stream, value.parent_index)
                || !read_value(stream, value.bind_local_pose.position.x)
                || !read_value(stream, value.bind_local_pose.position.y)
                || !read_value(stream, value.bind_local_pose.position.z)
                || !read_value(stream, value.bind_local_pose.rotation.x)
                || !read_value(stream, value.bind_local_pose.rotation.y)
                || !read_value(stream, value.bind_local_pose.rotation.z)
                || !read_value(stream, value.bind_local_pose.rotation.w))
            {
                return fail(error_message, "The .bflex bone data is truncated.");
            }
            bones.push_back(std::move(value));
        }

        if (!asset.rig.set_bones(std::move(bones), error_message))
        {
            return false;
        }

        std::vector<std::uint32_t> offsets(static_cast<std::size_t>(vertex_count) + 1);
        std::vector<std::uint32_t> bone_indices(influence_count);
        std::vector<float> weights(influence_count);
        if (!read_values(stream, offsets.data(), offsets.size() * sizeof(std::uint32_t))
            || !read_values(stream, bone_indices.data(), bone_indices.size() * sizeof(std::uint32_t))
            || !read_values(stream, weights.data(), weights.size() * sizeof(float)))
        {
            return fail(error_message, "The .bflex skinning weight data is truncated.");
        }

        char trailing_byte = 0;
        if (stream.read(&trailing_byte, 1))
        {
            return fail(error_message, "The .bflex file contains unexpected trailing data.");
        }

        if (!asset.weights.set_data(
            vertex_count,
            std::move(offsets),
            std::move(bone_indices),
            std::move(weights),
            bone_count,
            error_message))
        {
            return false;
        }

        output_asset = std::move(asset);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }
}
