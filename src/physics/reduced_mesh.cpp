#include <boneflex/physics/reduced_mesh.h>

#include <cmath>
#include <utility>

namespace boneflex
{
    bool reduced_mesh::set_data(
        std::vector<vec3> rest_positions,
        std::vector<std::uint32_t> triangle_indices,
        std::string* error_message
    )
    {
        const auto fail = [this, error_message](const char* message) -> bool
        {
            _rest_positions.clear();
            _triangle_indices.clear();
            _triangle_material_indices.clear();
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        };

        if (rest_positions.empty())
        {
            return fail("A reduced mesh requires at least one control point.");
        }
        for (const vec3& position : rest_positions)
        {
            if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
            {
                return fail("Reduced mesh positions must be finite.");
            }
        }
        if (triangle_indices.size() % 3 != 0)
        {
            return fail("Triangle index count must be divisible by three.");
        }
        for (std::size_t index = 0; index < triangle_indices.size(); index += 3)
        {
            const std::uint32_t a = triangle_indices[index];
            const std::uint32_t b = triangle_indices[index + 1];
            const std::uint32_t c = triangle_indices[index + 2];
            if (a >= rest_positions.size() || b >= rest_positions.size() || c >= rest_positions.size())
            {
                return fail("Reduced mesh triangle index is out of range.");
            }
            if (a == b || b == c || c == a)
            {
                return fail("Reduced mesh triangles must reference three distinct control points.");
            }
        }

        _rest_positions = std::move(rest_positions);
        _triangle_indices = std::move(triangle_indices);
        _triangle_material_indices.assign(_triangle_indices.size() / 3, 0);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }

    bool reduced_mesh::set_triangle_material_indices(std::vector<std::uint32_t> material_indices, std::string* error_message)
    {
        if (material_indices.size() != triangle_count())
        {
            if (error_message) *error_message = "Triangle material index count must match the reduced triangle count.";
            return false;
        }
        _triangle_material_indices = std::move(material_indices);
        if (error_message) error_message->clear();
        return true;
    }

    std::size_t reduced_mesh::control_point_count() const
    {
        return _rest_positions.size();
    }

    std::size_t reduced_mesh::triangle_count() const
    {
        return _triangle_indices.size() / 3;
    }

    const std::vector<vec3>& reduced_mesh::rest_positions() const
    {
        return _rest_positions;
    }

    const std::vector<std::uint32_t>& reduced_mesh::triangle_indices() const
    {
        return _triangle_indices;
    }

    const std::vector<std::uint32_t>& reduced_mesh::triangle_material_indices() const
    {
        return _triangle_material_indices;
    }

    bool vertex_embedding::set_data(
        std::size_t render_vertex_count,
        std::vector<std::uint32_t> offsets,
        std::vector<std::uint32_t> control_point_indices,
        std::vector<float> weights,
        std::size_t control_point_count,
        std::string* error_message
    )
    {
        const auto fail = [this, error_message](const char* message) -> bool
        {
            _render_vertex_count = 0;
            _offsets.clear();
            _control_point_indices.clear();
            _weights.clear();
            if (error_message != nullptr)
            {
                *error_message = message;
            }
            return false;
        };

        if (render_vertex_count == 0 || control_point_count == 0)
        {
            return fail("Render vertex and control point counts must be positive.");
        }
        if (offsets.size() != render_vertex_count + 1 || offsets.empty() || offsets.front() != 0)
        {
            return fail("Embedding offsets must contain render_vertex_count + 1 entries and start at zero.");
        }
        if (control_point_indices.size() != weights.size() || offsets.back() != weights.size())
        {
            return fail("Embedding indices, weights, and offsets have inconsistent sizes.");
        }

        for (std::size_t vertex_index = 0; vertex_index < render_vertex_count; ++vertex_index)
        {
            const std::uint32_t begin = offsets[vertex_index];
            const std::uint32_t end = offsets[vertex_index + 1];
            if (begin >= end || end > weights.size())
            {
                return fail("Every render vertex must have at least one valid embedding influence.");
            }

            float weight_sum = 0.0f;
            for (std::uint32_t influence_index = begin; influence_index < end; ++influence_index)
            {
                if (control_point_indices[influence_index] >= control_point_count)
                {
                    return fail("Embedding control point index is out of range.");
                }
                const float weight = weights[influence_index];
                if (!std::isfinite(weight) || weight < 0.0f)
                {
                    return fail("Embedding weights must be finite and non-negative.");
                }
                weight_sum += weight;
            }
            if (!std::isfinite(weight_sum) || weight_sum <= 1.0e-8f)
            {
                return fail("Every render vertex must have a positive finite embedding weight sum.");
            }
            for (std::uint32_t influence_index = begin; influence_index < end; ++influence_index)
            {
                weights[influence_index] /= weight_sum;
            }
        }

        _render_vertex_count = render_vertex_count;
        _offsets = std::move(offsets);
        _control_point_indices = std::move(control_point_indices);
        _weights = std::move(weights);
        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }

    std::size_t vertex_embedding::render_vertex_count() const
    {
        return _render_vertex_count;
    }

    std::size_t vertex_embedding::influence_count() const
    {
        return _weights.size();
    }

    const std::vector<std::uint32_t>& vertex_embedding::offsets() const
    {
        return _offsets;
    }

    const std::vector<std::uint32_t>& vertex_embedding::control_point_indices() const
    {
        return _control_point_indices;
    }

    const std::vector<float>& vertex_embedding::weights() const
    {
        return _weights;
    }

    bool reconstruct_render_vertices(
        const std::vector<vec3>& control_point_positions,
        const vertex_embedding& embedding,
        std::vector<vec3>& output_positions
    )
    {
        if (control_point_positions.empty()
            || embedding.render_vertex_count() == 0
            || embedding.offsets().size() != embedding.render_vertex_count() + 1
            || embedding.control_point_indices().size() != embedding.weights().size())
        {
            return false;
        }

        std::vector<vec3> result(embedding.render_vertex_count());
        for (std::size_t vertex_index = 0; vertex_index < result.size(); ++vertex_index)
        {
            vec3 position;
            for (std::uint32_t influence_index = embedding.offsets()[vertex_index];
                 influence_index < embedding.offsets()[vertex_index + 1];
                 ++influence_index)
            {
                const std::size_t index = influence_index;
                const std::uint32_t control_point_index = embedding.control_point_indices()[index];
                if (control_point_index >= control_point_positions.size())
                {
                    return false;
                }
                position += control_point_positions[control_point_index] * embedding.weights()[index];
            }
            result[vertex_index] = position;
        }

        output_positions = std::move(result);
        return true;
    }
}
