#include <boneflex/physics/shape_matching_cluster_generation.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    bool test_cluster_generation_covers_surface()
    {
        constexpr std::uint32_t side = 20;
        std::vector<boneflex::vec3> positions;
        std::vector<std::uint32_t> triangles;
        positions.reserve(side * side);

        for (std::uint32_t y = 0; y < side; ++y)
        {
            for (std::uint32_t x = 0; x < side; ++x)
            {
                positions.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
            }
        }

        for (std::uint32_t y = 0; y + 1 < side; ++y)
        {
            for (std::uint32_t x = 0; x + 1 < side; ++x)
            {
                const std::uint32_t a = y * side + x;
                const std::uint32_t b = a + 1;
                const std::uint32_t c = a + side;
                const std::uint32_t d = c + 1;
                triangles.insert(triangles.end(), {a, b, c, b, d, c});
            }
        }

        boneflex::reduced_mesh mesh;
        if (!mesh.set_data(std::move(positions), std::move(triangles)))
        {
            return false;
        }

        boneflex::shape_matching_cluster_options options;
        options.cluster_size = 24;
        options.overlap_ratio = 0.5f;

        boneflex::shape_matching_clusters clusters;
        std::string error_message;
        if (!boneflex::generate_shape_matching_clusters(mesh, options, clusters, &error_message))
        {
            std::cerr << error_message << '\n';
            return false;
        }

        if (clusters.cluster_count() < 2
            || clusters.control_point_count() != mesh.control_point_count())
        {
            return false;
        }

        std::vector<std::uint32_t> coverage(mesh.control_point_count(), 0);
        const std::vector<std::uint32_t>& offsets = clusters.offsets();
        const std::vector<std::uint32_t>& indices = clusters.control_point_indices();
        for (std::size_t cluster_index = 0; cluster_index < clusters.cluster_count(); ++cluster_index)
        {
            const std::uint32_t begin = offsets[cluster_index];
            const std::uint32_t end = offsets[cluster_index + 1];
            if (begin >= end || end - begin > options.cluster_size)
            {
                return false;
            }
            for (std::uint32_t i = begin; i < end; ++i)
            {
                if (indices[i] >= coverage.size())
                {
                    return false;
                }
                ++coverage[indices[i]];
            }
        }

        bool has_overlap = false;
        for (std::uint32_t count : coverage)
        {
            if (count == 0)
            {
                return false;
            }
            if (count > 1)
            {
                has_overlap = true;
            }
        }
        return has_overlap;
    }

    bool test_invalid_options_preserve_output()
    {
        boneflex::reduced_mesh mesh;
        if (!mesh.set_data({{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0, 1, 2, 0, 3, 1}))
        {
            return false;
        }

        boneflex::shape_matching_cluster_options valid_options;
        valid_options.cluster_size = 4;
        boneflex::shape_matching_clusters clusters;
        if (!boneflex::generate_shape_matching_clusters(mesh, valid_options, clusters))
        {
            return false;
        }
        const std::size_t previous_cluster_count = clusters.cluster_count();

        boneflex::shape_matching_cluster_options invalid_options;
        invalid_options.cluster_size = 2;
        std::string error_message;
        const bool success = boneflex::generate_shape_matching_clusters(
            mesh, invalid_options, clusters, &error_message);
        return !success && !error_message.empty()
            && clusters.cluster_count() == previous_cluster_count;
    }
}

int main()
{
    if (!test_cluster_generation_covers_surface())
    {
        std::cerr << "Cluster generation coverage test failed.\n";
        return 1;
    }
    if (!test_invalid_options_preserve_output())
    {
        std::cerr << "Invalid options preservation test failed.\n";
        return 1;
    }
    std::cout << "All shape matching cluster generation tests passed.\n";
    return 0;
}
