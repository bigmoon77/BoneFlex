#include <boneflex/physics/mesh_reduction.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
    using clock_type = std::chrono::steady_clock;

    double elapsed_milliseconds(clock_type::time_point begin, clock_type::time_point end)
    {
        return std::chrono::duration<double, std::milli>(end - begin).count();
    }

    void build_uv_sphere(
        std::uint32_t longitude_segments,
        std::uint32_t latitude_segments,
        std::vector<boneflex::vec3>& positions,
        std::vector<std::uint32_t>& indices)
    {
        positions.clear();
        indices.clear();
        positions.reserve(static_cast<std::size_t>(longitude_segments) * latitude_segments);
        constexpr float pi = 3.14159265358979323846f;
        for (std::uint32_t y = 0; y < latitude_segments; ++y)
        {
            const float latitude = pi * static_cast<float>(y) / static_cast<float>(latitude_segments - 1);
            const float sin_latitude = std::sin(latitude);
            const float cos_latitude = std::cos(latitude);
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const float longitude = 2.0f * pi * static_cast<float>(x) / static_cast<float>(longitude_segments);
                positions.emplace_back(sin_latitude * std::cos(longitude), cos_latitude, sin_latitude * std::sin(longitude));
            }
        }
        indices.reserve(static_cast<std::size_t>(longitude_segments) * (latitude_segments - 1) * 6);
        for (std::uint32_t y = 0; y + 1 < latitude_segments; ++y)
        {
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const std::uint32_t next_x = (x + 1) % longitude_segments;
                const std::uint32_t a = y * longitude_segments + x;
                const std::uint32_t b = y * longitude_segments + next_x;
                const std::uint32_t c = (y + 1) * longitude_segments + x;
                const std::uint32_t d = (y + 1) * longitude_segments + next_x;
                indices.insert(indices.end(), {a, c, b, b, c, d});
            }
        }
    }
}

int main(int argc, char** argv)
{
    std::uint32_t longitude_segments = 900;
    std::uint32_t latitude_segments = 500;
    std::size_t target_control_points = 5000;
    if (argc > 1) target_control_points = static_cast<std::size_t>(std::stoull(argv[1]));
    if (argc > 2) longitude_segments = static_cast<std::uint32_t>(std::stoul(argv[2]));
    if (argc > 3) latitude_segments = static_cast<std::uint32_t>(std::stoul(argv[3]));
    if (longitude_segments < 3 || latitude_segments < 3 || target_control_points == 0)
    {
        std::cerr << "Usage: boneflex_mesh_reduction_benchmark [target_control_points] [longitude_segments] [latitude_segments]\n";
        return 1;
    }

    std::vector<boneflex::vec3> render_positions;
    std::vector<std::uint32_t> render_indices;
    const clock_type::time_point mesh_begin = clock_type::now();
    build_uv_sphere(longitude_segments, latitude_segments, render_positions, render_indices);
    const clock_type::time_point mesh_end = clock_type::now();

    boneflex::mesh_reduction_options options;
    options.target_control_point_count = target_control_points;
    options.embedding_influence_count = 4;
    boneflex::reduced_mesh reduced;
    boneflex::vertex_embedding embedding;
    std::string error_message;
    const clock_type::time_point reduction_begin = clock_type::now();
    const bool success = boneflex::generate_reduced_mesh(render_positions, render_indices, options, reduced, embedding, &error_message);
    const clock_type::time_point reduction_end = clock_type::now();
    if (!success)
    {
        std::cerr << "Mesh reduction failed: " << error_message << '\n';
        return 1;
    }

    std::vector<boneflex::vec3> reconstructed;
    const clock_type::time_point reconstruction_begin = clock_type::now();
    if (!boneflex::reconstruct_render_vertices(reduced.rest_positions(), embedding, reconstructed))
    {
        std::cerr << "Render vertex reconstruction failed.\n";
        return 1;
    }
    const clock_type::time_point reconstruction_end = clock_type::now();

    double squared_error_sum = 0.0;
    double maximum_error = 0.0;
    for (std::size_t i = 0; i < render_positions.size(); ++i)
    {
        const boneflex::vec3 difference = reconstructed[i] - render_positions[i];
        const double error = std::sqrt(static_cast<double>(difference.length_squared()));
        squared_error_sum += error * error;
        maximum_error = std::max(maximum_error, error);
    }
    const double rms_error = std::sqrt(squared_error_sum / static_cast<double>(render_positions.size()));
    const double control_point_ratio = static_cast<double>(reduced.control_point_count()) / static_cast<double>(render_positions.size());
    const std::size_t embedding_bytes = embedding.offsets().size() * sizeof(std::uint32_t)
        + embedding.control_point_indices().size() * sizeof(std::uint32_t)
        + embedding.weights().size() * sizeof(float);
    const std::size_t reduced_bytes = reduced.rest_positions().size() * sizeof(boneflex::vec3)
        + reduced.triangle_indices().size() * sizeof(std::uint32_t);

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "BoneFlex mesh reduction benchmark\n";
    std::cout << "Render vertices:          " << render_positions.size() << '\n';
    std::cout << "Render triangles:         " << render_indices.size() / 3 << '\n';
    std::cout << "Target control points:    " << target_control_points << '\n';
    std::cout << "Generated control points: " << reduced.control_point_count() << '\n';
    std::cout << "Reduced triangles:        " << reduced.triangle_count() << '\n';
    std::cout << "Control point ratio:      " << control_point_ratio * 100.0 << "%\n";
    std::cout << "Embedding influences:     " << embedding.influence_count() << '\n';
    std::cout << "RMS reconstruction error: " << rms_error << '\n';
    std::cout << "Max reconstruction error: " << maximum_error << '\n';
    std::cout << "Mesh generation:          " << elapsed_milliseconds(mesh_begin, mesh_end) << " ms\n";
    std::cout << "Reduction + embedding:    " << elapsed_milliseconds(reduction_begin, reduction_end) << " ms\n";
    std::cout << "Reconstruction:           " << elapsed_milliseconds(reconstruction_begin, reconstruction_end) << " ms\n";
    std::cout << "Reduced mesh storage:     " << reduced_bytes / 1048576.0 << " MiB\n";
    std::cout << "Embedding storage:        " << embedding_bytes / 1048576.0 << " MiB\n";
    return 0;
}
