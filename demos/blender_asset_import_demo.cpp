#include <boneflex/io/blender_asset.h>
#include <boneflex/physics/mesh_reduction.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: boneflex_blender_asset_demo <asset.bflex> [target_control_points]\n";
        return EXIT_FAILURE;
    }

    boneflex::blender_asset asset;
    std::string error_message;
    if (!boneflex::load_blender_asset(argv[1], asset, &error_message))
    {
        std::cerr << "Failed to load asset: " << error_message << '\n';
        return EXIT_FAILURE;
    }

    std::vector<boneflex::vec3> skinned_positions;
    if (!asset.skin_current_pose(skinned_positions))
    {
        std::cerr << "Failed to evaluate the imported bind pose.\n";
        return EXIT_FAILURE;
    }

    std::size_t target_control_points = 5000;
    if (argc >= 3)
    {
        try
        {
            target_control_points = static_cast<std::size_t>(std::stoull(argv[2]));
        }
        catch (...)
        {
            std::cerr << "Invalid target control point count.\n";
            return EXIT_FAILURE;
        }
    }

    boneflex::mesh_reduction_options reduction_options;
    reduction_options.target_control_point_count = target_control_points;
    boneflex::reduced_mesh reduced_mesh;
    boneflex::vertex_embedding embedding;
    if (!boneflex::generate_reduced_mesh(
        skinned_positions,
        asset.triangle_indices,
        reduction_options,
        reduced_mesh,
        embedding,
        &error_message))
    {
        std::cerr << "Mesh reduction failed: " << error_message << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "BoneFlex Blender asset import demo\n";
    std::cout << "Vertices:             " << asset.bind_positions.size() << '\n';
    std::cout << "Triangles:            " << asset.triangle_indices.size() / 3 << '\n';
    std::cout << "Bones:                " << asset.rig.bone_count() << '\n';
    std::cout << "Skinning influences:  " << asset.weights.influence_count() << '\n';
    std::cout << "Reduced control pts:  " << reduced_mesh.control_point_count() << '\n';
    std::cout << "Reduced triangles:    " << reduced_mesh.triangle_count() << '\n';
    std::cout << "Embedding influences: " << embedding.influence_count() << '\n';
    std::cout << "Bind-pose skinning:   passed\n";
    return EXIT_SUCCESS;
}
