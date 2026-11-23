#include <boneflex/io/blender_asset.h>

#include <array>
#include <cstdint>
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

    bool write_test_asset(const std::filesystem::path& path)
    {
        std::ofstream stream(path, std::ios::binary);
        if (!stream)
        {
            return false;
        }

        stream.write("BFLX", 4);
        const std::array<std::uint32_t, 5> header = { 1, 3, 3, 1, 3 };
        for (const std::uint32_t value : header)
        {
            write_value(stream, value);
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
    if (!write_test_asset(path))
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
    std::filesystem::remove(path);

    if (asset.bind_positions.size() != 3
        || asset.triangle_indices.size() != 3
        || asset.rig.bone_count() != 1
        || asset.weights.vertex_count() != 3)
    {
        std::cerr << "Imported asset counts do not match the fixture.\n";
        return 1;
    }

    std::vector<boneflex::vec3> skinned_positions;
    if (!asset.skin_current_pose(skinned_positions) || skinned_positions.size() != 3)
    {
        std::cerr << "Bind-pose skinning failed.\n";
        return 1;
    }

    for (std::size_t index = 0; index < skinned_positions.size(); ++index)
    {
        if ((skinned_positions[index] - asset.bind_positions[index]).length() > 1.0e-5f)
        {
            std::cerr << "Bind-pose skinning changed a vertex position.\n";
            return 1;
        }
    }

    std::cout << "Blender asset import tests passed.\n";
    return 0;
}
