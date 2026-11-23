#include <boneflex/physics/shape_matching_cluster_generation.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace boneflex
{
    namespace
    {
        bool fail(std::string* error_message, const char* message)
        {
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        }

        bool finite_vec3(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        void build_adjacency(
            const reduced_mesh& mesh,
            std::vector<std::vector<std::uint32_t>>& adjacency)
        {
            const std::size_t point_count = mesh.control_point_count();
            adjacency.clear();
            adjacency.resize(point_count);

            const std::vector<std::uint32_t>& triangles = mesh.triangle_indices();
            for (std::size_t i = 0; i < triangles.size(); i += 3)
            {
                const std::uint32_t a = triangles[i];
                const std::uint32_t b = triangles[i + 1];
                const std::uint32_t c = triangles[i + 2];
                adjacency[a].push_back(b);
                adjacency[a].push_back(c);
                adjacency[b].push_back(a);
                adjacency[b].push_back(c);
                adjacency[c].push_back(a);
                adjacency[c].push_back(b);
            }

            for (std::vector<std::uint32_t>& neighbors : adjacency)
            {
                std::sort(neighbors.begin(), neighbors.end());
                neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
            }
        }

        std::vector<std::uint32_t> gather_cluster(
            std::uint32_t seed,
            std::size_t cluster_size,
            const std::vector<std::vector<std::uint32_t>>& adjacency,
            std::vector<std::uint32_t>& visit_stamps,
            std::uint32_t& visit_stamp)
        {
            ++visit_stamp;
            if (visit_stamp == 0)
            {
                std::fill(visit_stamps.begin(), visit_stamps.end(), 0);
                visit_stamp = 1;
            }

            std::vector<std::uint32_t> cluster;
            cluster.reserve(cluster_size);
            std::queue<std::uint32_t> pending;
            pending.push(seed);
            visit_stamps[seed] = visit_stamp;

            while (!pending.empty() && cluster.size() < cluster_size)
            {
                const std::uint32_t current = pending.front();
                pending.pop();
                cluster.push_back(current);

                for (std::uint32_t neighbor : adjacency[current])
                {
                    if (visit_stamps[neighbor] == visit_stamp)
                    {
                        continue;
                    }
                    visit_stamps[neighbor] = visit_stamp;
                    pending.push(neighbor);
                }
            }
            return cluster;
        }
    }

    bool generate_shape_matching_clusters(
        const reduced_mesh& mesh,
        const shape_matching_cluster_options& options,
        shape_matching_clusters& output_clusters,
        std::string* error_message)
    {
        const std::size_t point_count = mesh.control_point_count();
        if (point_count == 0 || point_count > std::numeric_limits<std::uint32_t>::max())
        {
            return fail(error_message, "Reduced mesh must contain between 1 and UINT32_MAX control points.");
        }
        if (options.cluster_size < 4 || options.cluster_size > point_count
            || !std::isfinite(options.overlap_ratio)
            || options.overlap_ratio < 0.0f || options.overlap_ratio >= 1.0f)
        {
            return fail(error_message, "Cluster size must be in [4, control point count] and overlap ratio must be in [0, 1).");
        }

        const std::vector<vec3>& positions = mesh.rest_positions();
        for (const vec3& position : positions)
        {
            if (!finite_vec3(position))
            {
                return fail(error_message, "Reduced mesh positions must be finite.");
            }
        }

        std::vector<std::vector<std::uint32_t>> adjacency;
        build_adjacency(mesh, adjacency);

        const double advance = static_cast<double>(options.cluster_size)
            * (1.0 - static_cast<double>(options.overlap_ratio));
        const std::size_t seed_count = std::max<std::size_t>(
            1,
            static_cast<std::size_t>(std::ceil(static_cast<double>(point_count) / advance)));

        // Deterministic Euclidean farthest-point sampling spreads cluster seeds
        // over the surface. Cluster membership itself is determined by topology.
        std::vector<std::uint32_t> seeds;
        seeds.reserve(std::min(seed_count, point_count));
        std::vector<float> nearest_distance_squared(point_count, std::numeric_limits<float>::max());
        std::vector<bool> selected(point_count, false);

        std::uint32_t seed = 0;
        for (std::size_t seed_index = 0; seed_index < std::min(seed_count, point_count); ++seed_index)
        {
            seeds.push_back(seed);
            selected[seed] = true;
            for (std::size_t point_index = 0; point_index < point_count; ++point_index)
            {
                const float distance_squared = (positions[point_index] - positions[seed]).length_squared();
                nearest_distance_squared[point_index] = std::min(
                    nearest_distance_squared[point_index], distance_squared);
            }

            float farthest_distance = -1.0f;
            std::uint32_t farthest_index = 0;
            for (std::size_t point_index = 0; point_index < point_count; ++point_index)
            {
                if (!selected[point_index] && nearest_distance_squared[point_index] > farthest_distance)
                {
                    farthest_distance = nearest_distance_squared[point_index];
                    farthest_index = static_cast<std::uint32_t>(point_index);
                }
            }
            seed = farthest_index;
        }

        std::vector<std::uint32_t> offsets;
        std::vector<std::uint32_t> indices;
        std::vector<bool> covered(point_count, false);
        std::vector<std::uint32_t> visit_stamps(point_count, 0);
        std::uint32_t visit_stamp = 0;
        offsets.reserve(seeds.size() + point_count + 1);
        indices.reserve(seeds.size() * options.cluster_size);
        offsets.push_back(0);

        const auto append_cluster = [&](std::uint32_t cluster_seed) -> bool
        {
            const std::vector<std::uint32_t> cluster = gather_cluster(
                cluster_seed, options.cluster_size, adjacency, visit_stamps, visit_stamp);
            if (cluster.empty())
            {
                return false;
            }
            indices.insert(indices.end(), cluster.begin(), cluster.end());
            for (std::uint32_t point_index : cluster)
            {
                covered[point_index] = true;
            }
            if (indices.size() > std::numeric_limits<std::uint32_t>::max())
            {
                return false;
            }
            offsets.push_back(static_cast<std::uint32_t>(indices.size()));
            return true;
        };

        for (std::uint32_t cluster_seed : seeds)
        {
            if (!append_cluster(cluster_seed))
            {
                return fail(error_message, "Failed to generate a shape matching cluster.");
            }
        }

        // Disconnected components and isolated control points may not be reached
        // from the initial seeds. Add local clusters until coverage is complete.
        for (std::size_t point_index = 0; point_index < point_count; ++point_index)
        {
            if (!covered[point_index] && !append_cluster(static_cast<std::uint32_t>(point_index)))
            {
                return fail(error_message, "Failed to cover every reduced control point.");
            }
        }

        shape_matching_clusters generated_clusters;
        if (!generated_clusters.set_data(
            point_count, std::move(offsets), std::move(indices), error_message))
        {
            return false;
        }

        output_clusters = std::move(generated_clusters);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }
}
