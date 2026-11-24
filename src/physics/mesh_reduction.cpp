#include <boneflex/physics/mesh_reduction.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace boneflex
{
    namespace
    {
        struct cell_key
        {
            std::int64_t x = 0;
            std::int64_t y = 0;
            std::int64_t z = 0;

            bool operator==(const cell_key& other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct cell_key_hash
        {
            std::size_t operator()(const cell_key& key) const
            {
                const std::size_t hx = std::hash<std::int64_t>{}(key.x);
                const std::size_t hy = std::hash<std::int64_t>{}(key.y);
                const std::size_t hz = std::hash<std::int64_t>{}(key.z);
                return hx ^ (hy + 0x9e3779b9u + (hx << 6) + (hx >> 2))
                    ^ (hz + 0x9e3779b9u + (hy << 6) + (hy >> 2));
            }
        };

        struct cell_group
        {
            vec3 sum;
            std::size_t count = 0;
        };

        struct triangle_key
        {
            std::uint32_t a = 0;
            std::uint32_t b = 0;
            std::uint32_t c = 0;
            std::uint32_t material = 0;

            bool operator==(const triangle_key& other) const
            {
                return a == other.a && b == other.b && c == other.c && material == other.material;
            }
        };

        struct triangle_key_hash
        {
            std::size_t operator()(const triangle_key& key) const
            {
                std::size_t hash = key.a;
                hash = hash * 16777619u ^ key.b;
                hash = hash * 16777619u ^ key.c;
                hash = hash * 16777619u ^ key.material;
                return hash;
            }
        };

        cell_key get_cell_key(const vec3& position, const vec3& minimum, float cell_size)
        {
            return cell_key{
                static_cast<std::int64_t>(std::floor((position.x - minimum.x) / cell_size)),
                static_cast<std::int64_t>(std::floor((position.y - minimum.y) / cell_size)),
                static_cast<std::int64_t>(std::floor((position.z - minimum.z) / cell_size))
            };
        }

        std::size_t count_occupied_cells(
            const std::vector<vec3>& positions,
            const vec3& minimum,
            float cell_size)
        {
            std::unordered_set<cell_key, cell_key_hash> cells;
            cells.reserve(positions.size());
            for (const vec3& position : positions)
            {
                cells.insert(get_cell_key(position, minimum, cell_size));
            }
            return cells.size();
        }

        bool finite_vec3(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool fail(std::string* error_message, const char* message)
        {
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        }
    }

    bool generate_reduced_mesh(
        const std::vector<vec3>& render_positions,
        const std::vector<std::uint32_t>& render_triangle_indices,
        const mesh_reduction_options& options,
        reduced_mesh& output_mesh,
        vertex_embedding& output_embedding,
        std::string* error_message,
        const std::vector<std::uint32_t>& render_triangle_material_indices)
    {
        if (render_positions.empty() || render_positions.size() > std::numeric_limits<std::uint32_t>::max())
        {
            return fail(error_message, "Render mesh must contain between 1 and UINT32_MAX vertices.");
        }
        if (render_triangle_indices.empty() || render_triangle_indices.size() % 3 != 0)
        {
            return fail(error_message, "Render mesh must contain a non-empty triangle index list.");
        }
        if (!render_triangle_material_indices.empty()
            && render_triangle_material_indices.size() != render_triangle_indices.size() / 3)
            return fail(error_message, "Render triangle material count must match the triangle count.");
        if (options.target_control_point_count == 0 || options.embedding_influence_count == 0
            || options.embedding_influence_count > 16
            || !std::isfinite(options.embedding_support_radius_scale)
            || options.embedding_support_radius_scale <= 0.0f)
        {
            return fail(error_message, "Target control point count must be positive and embedding influences must be in [1, 16].");
        }
        for (const vec3& position : render_positions)
        {
            if (!finite_vec3(position))
            {
                return fail(error_message, "Render positions must be finite.");
            }
        }
        for (std::uint32_t index : render_triangle_indices)
        {
            if (index >= render_positions.size())
            {
                return fail(error_message, "Render triangle index is out of range.");
            }
        }

        vec3 minimum = render_positions.front();
        vec3 maximum = render_positions.front();
        for (const vec3& position : render_positions)
        {
            minimum.x = std::min(minimum.x, position.x);
            minimum.y = std::min(minimum.y, position.y);
            minimum.z = std::min(minimum.z, position.z);
            maximum.x = std::max(maximum.x, position.x);
            maximum.y = std::max(maximum.y, position.y);
            maximum.z = std::max(maximum.z, position.z);
        }

        const vec3 extent = maximum - minimum;
        const float maximum_extent = std::max(extent.x, std::max(extent.y, extent.z));
        if (maximum_extent <= 1.0e-8f)
        {
            return fail(error_message, "Render mesh has no spatial extent.");
        }

        const std::size_t target_count = std::min(options.target_control_point_count, render_positions.size());
        float low = maximum_extent * 1.0e-6f;
        float high = maximum_extent;
        float best_cell_size = high;
        std::size_t best_count = 1;
        std::size_t best_difference = std::numeric_limits<std::size_t>::max();

        for (int iteration = 0; iteration < 28; ++iteration)
        {
            const float cell_size = (low + high) * 0.5f;
            const std::size_t count = count_occupied_cells(render_positions, minimum, cell_size);
            const std::size_t difference = count > target_count ? count - target_count : target_count - count;
            if (difference < best_difference)
            {
                best_difference = difference;
                best_cell_size = cell_size;
                best_count = count;
            }
            if (count > target_count)
            {
                low = cell_size;
            }
            else
            {
                high = cell_size;
            }
        }
        (void)best_count;

        std::unordered_map<cell_key, std::uint32_t, cell_key_hash> cell_indices;
        cell_indices.reserve(target_count * 2 + 1);
        std::vector<cell_group> groups;
        std::vector<cell_key> vertex_cells(render_positions.size());
        for (std::size_t vertex_index = 0; vertex_index < render_positions.size(); ++vertex_index)
        {
            const cell_key key = get_cell_key(render_positions[vertex_index], minimum, best_cell_size);
            vertex_cells[vertex_index] = key;
            const auto found = cell_indices.find(key);
            std::uint32_t group_index = 0;
            if (found == cell_indices.end())
            {
                group_index = static_cast<std::uint32_t>(groups.size());
                cell_indices.emplace(key, group_index);
                groups.emplace_back();
            }
            else
            {
                group_index = found->second;
            }
            groups[group_index].sum += render_positions[vertex_index];
            ++groups[group_index].count;
        }

        std::vector<vec3> control_positions(groups.size());
        for (std::size_t group_index = 0; group_index < groups.size(); ++group_index)
        {
            control_positions[group_index] = groups[group_index].sum / static_cast<float>(groups[group_index].count);
        }

        std::vector<std::uint32_t> reduced_triangles;
        std::vector<std::uint32_t> reduced_material_indices;
        reduced_triangles.reserve(render_triangle_indices.size());
        reduced_material_indices.reserve(render_triangle_indices.size() / 3);
        std::unordered_set<triangle_key, triangle_key_hash> unique_triangles;
        unique_triangles.reserve(render_triangle_indices.size() / 3);
        for (std::size_t triangle_index = 0; triangle_index < render_triangle_indices.size(); triangle_index += 3)
        {
            const std::uint32_t a = cell_indices.at(vertex_cells[render_triangle_indices[triangle_index]]);
            const std::uint32_t b = cell_indices.at(vertex_cells[render_triangle_indices[triangle_index + 1]]);
            const std::uint32_t c = cell_indices.at(vertex_cells[render_triangle_indices[triangle_index + 2]]);
            if (a == b || b == c || c == a)
            {
                continue;
            }
            const std::uint32_t material = render_triangle_material_indices.empty() ? 0u : render_triangle_material_indices[triangle_index / 3];
            triangle_key key{a, b, c, material};
            std::uint32_t sorted[3]{a, b, c};
            std::sort(sorted, sorted + 3);
            key = triangle_key{sorted[0], sorted[1], sorted[2], material};
            if (unique_triangles.insert(key).second)
            {
                reduced_triangles.push_back(a);
                reduced_triangles.push_back(b);
                reduced_triangles.push_back(c);
                reduced_material_indices.push_back(render_triangle_material_indices.empty()
                    ? 0u : render_triangle_material_indices[triangle_index / 3]);
            }
        }

        std::vector<std::uint32_t> embedding_offsets;
        std::vector<std::uint32_t> embedding_indices;
        std::vector<float> embedding_weights;
        embedding_offsets.reserve(render_positions.size() + 1);
        embedding_offsets.push_back(0);

        // A compact Wendland C2 kernel gives each control point a smoothly
        // vanishing influence at the support boundary. This avoids the hard
        // nearest-K switches produced by inverse-distance top-K embedding.
        const float base_support_radius = best_cell_size * options.embedding_support_radius_scale;
        const float base_support_radius_squared = base_support_radius * base_support_radius;
        const std::int64_t search_radius_cells =
            static_cast<std::int64_t>(std::ceil(options.embedding_support_radius_scale)) + 1;

        for (std::size_t vertex_index = 0; vertex_index < render_positions.size(); ++vertex_index)
        {
            const cell_key center = vertex_cells[vertex_index];
            std::vector<std::pair<float, std::uint32_t>> candidates;
            candidates.reserve(128);
            for (std::int64_t z = center.z - search_radius_cells; z <= center.z + search_radius_cells; ++z)
            {
                for (std::int64_t y = center.y - search_radius_cells; y <= center.y + search_radius_cells; ++y)
                {
                    for (std::int64_t x = center.x - search_radius_cells; x <= center.x + search_radius_cells; ++x)
                    {
                        const auto found = cell_indices.find(cell_key{x, y, z});
                        if (found == cell_indices.end())
                        {
                            continue;
                        }
                        const std::uint32_t control_index = found->second;
                        const float distance_squared =
                            (render_positions[vertex_index] - control_positions[control_index]).length_squared();
                        candidates.emplace_back(distance_squared, control_index);
                    }
                }
            }
            std::sort(candidates.begin(), candidates.end(),
                [](const std::pair<float, std::uint32_t>& left, const std::pair<float, std::uint32_t>& right)
                {
                    if (left.first != right.first) return left.first < right.first;
                    return left.second < right.second;
                });
            if (candidates.empty())
            {
                return fail(error_message, "Failed to find a local control point for a render vertex.");
            }

            // Keep at least the requested number of influences for sparse
            // regions by extending the support only when necessary.
            float support_radius_squared = base_support_radius_squared;
            std::size_t supported_count = 0;
            while (supported_count < candidates.size()
                && candidates[supported_count].first < support_radius_squared)
            {
                ++supported_count;
            }
            if (supported_count < options.embedding_influence_count)
            {
                const std::size_t minimum_count =
                    std::min(options.embedding_influence_count, candidates.size());
                support_radius_squared = std::max(
                    support_radius_squared,
                    candidates[minimum_count - 1].first * 1.0001f + 1.0e-12f);
            }

            float weight_sum = 0.0f;
            const std::size_t first_index = embedding_indices.size();
            for (const auto& candidate : candidates)
            {
                if (candidate.first >= support_radius_squared)
                {
                    break;
                }

                const float normalized_distance =
                    std::sqrt(candidate.first / support_radius_squared);
                const float remaining = 1.0f - normalized_distance;
                const float remaining_squared = remaining * remaining;
                const float weight = remaining_squared * remaining_squared
                    * (4.0f * normalized_distance + 1.0f);
                if (weight <= 0.0f)
                {
                    continue;
                }

                embedding_indices.push_back(candidate.second);
                embedding_weights.push_back(weight);
                weight_sum += weight;
            }

            if (embedding_indices.size() == first_index || weight_sum <= 1.0e-12f)
            {
                embedding_indices.push_back(candidates.front().second);
                embedding_weights.push_back(1.0f);
            }
            else
            {
                for (std::size_t influence_index = first_index;
                    influence_index < embedding_weights.size(); ++influence_index)
                {
                    embedding_weights[influence_index] /= weight_sum;
                }
            }

            if (embedding_indices.size() > std::numeric_limits<std::uint32_t>::max())
            {
                return fail(error_message, "Embedding influence count exceeds UINT32_MAX.");
            }
            embedding_offsets.push_back(static_cast<std::uint32_t>(embedding_indices.size()));
        }

        reduced_mesh generated_mesh;
        vertex_embedding generated_embedding;
        if (!generated_mesh.set_data(std::move(control_positions), std::move(reduced_triangles), error_message)
            || !generated_mesh.set_triangle_material_indices(std::move(reduced_material_indices), error_message))
        {
            return false;
        }
        if (!generated_embedding.set_data(
            render_positions.size(),
            std::move(embedding_offsets),
            std::move(embedding_indices),
            std::move(embedding_weights),
            generated_mesh.control_point_count(),
            error_message))
        {
            return false;
        }

        output_mesh = std::move(generated_mesh);
        output_embedding = std::move(generated_embedding);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }
}
