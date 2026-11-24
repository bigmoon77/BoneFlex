#include <boneflex/io/blender_asset.h>

#include <array>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    template<typename value_type>
    void write_value(std::ostream& stream, const value_type& value)
    {
        stream.write(reinterpret_cast<const char*>(&value), sizeof(value_type));
    }

    bool write_test_asset(const std::filesystem::path& path, std::uint32_t version)
    {
        std::ofstream stream(path, std::ios::binary);
        if (!stream)
        {
            return false;
        }

        stream.write("BFLX", 4);
        const std::array<std::uint32_t, 6> header_v2 = { 2, 3, 3, 1, 3, 1 };
        const std::array<std::uint32_t, 5> header_v1 = { 1, 3, 3, 1, 3 };
        if (version == 1)
        {
            for (const std::uint32_t value : header_v1) write_value(stream, value);
        }
        else
        {
            for (const std::uint32_t value : header_v2) write_value(stream, value);
        }

        const std::array<float, 9> positions = {
            0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f
        };
        for (const float value : positions)
        {
            write_value(stream, value);
        }

        const std::array<std::uint32_t, 3> triangles = { 0, 1, 2 };
        for (const std::uint32_t value : triangles)
        {
            write_value(stream, value);
        }
        if (version >= 2)
        {
            const std::uint32_t material_index = 0;
            write_value(stream, material_index);
            const std::uint32_t material_name_length = 4;
            write_value(stream, material_name_length);
            stream.write("Body", 4);
            const std::array<float, 4> material_values = { 0.7f, 0.0001f, 0.98f, 0.8f };
            for (const float value : material_values) write_value(stream, value);
        }

        const std::uint32_t name_length = 4;
        write_value(stream, name_length);
        stream.write("Root", 4);
        const std::int32_t parent_index = -1;
        write_value(stream, parent_index);

        const std::array<float, 7> bind_pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        for (const float value : bind_pose)
        {
            write_value(stream, value);
        }

        const std::array<std::uint32_t, 4> offsets = { 0, 1, 2, 3 };
        const std::array<std::uint32_t, 3> bone_indices = { 0, 0, 0 };
        const std::array<float, 3> weights = { 1.0f, 1.0f, 1.0f };
        for (const std::uint32_t value : offsets) write_value(stream, value);
        for (const std::uint32_t value : bone_indices) write_value(stream, value);
        for (const float value : weights) write_value(stream, value);

        return static_cast<bool>(stream);
    }
}

int main()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "boneflex_blender_asset_test.bflex";
    for (const std::uint32_t version : { 1u, 2u })
    {
        if (!write_test_asset(path, version))
        {
            std::cerr << "Failed to write the test asset.\n";
            return 1;
        }

        boneflex::blender_asset asset;
        std::string error_message;
        if (!boneflex::load_blender_asset(path.string(), asset, &error_message))
        {
            std::filesystem::remove(path);
            std::cerr << "Asset load failed: " << error_message << '\n';
            return 1;
        }
        if (asset.bind_positions.size() != 3
            || asset.triangle_indices.size() != 3
            || asset.rig.bone_count() != 1
            || asset.weights.vertex_count() != 3
            || asset.triangle_material_indices.size() != 1
            || asset.materials.size() != 1)
        {
            std::filesystem::remove(path);
            std::cerr << "Imported asset counts do not match the fixture.\n";
            return 1;
        }
        if (version == 1 && asset.materials[0].name != "Default")
        {
            std::filesystem::remove(path);
            std::cerr << "Version 1 default material was not generated.\n";
            return 1;
        }
        if (version == 2 && (asset.materials[0].name != "Body"
            || std::abs(asset.materials[0].shape_matching_stiffness - 0.7f) > 1.0e-6f
            || std::abs(asset.materials[0].xpbd_compliance - 0.0001f) > 1.0e-7f))
        {
            std::filesystem::remove(path);
            std::cerr << "Version 2 material physics data was not imported.\n";
            return 1;
        }

        std::vector<boneflex::vec3> skinned_positions;
        if (!asset.skin_current_pose(skinned_positions) || skinned_positions.size() != 3)
        {
            std::filesystem::remove(path);
            std::cerr << "Bind-pose skinning failed.\n";
            return 1;
        }
        for (std::size_t index = 0; index < skinned_positions.size(); ++index)
        {
            if ((skinned_positions[index] - asset.bind_positions[index]).length() > 1.0e-5f)
            {
                std::filesystem::remove(path);
                std::cerr << "Bind-pose skinning changed a vertex position.\n";
                return 1;
            }
        }
    }
    std::filesystem::remove(path);

    std::cout << "Blender asset import tests passed.\n";
    return 0;
}
