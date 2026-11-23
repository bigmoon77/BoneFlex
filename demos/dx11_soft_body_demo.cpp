#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <windowsx.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <DirectXMath.h>
#include <wrl/client.h>

#include <boneflex/io/blender_asset.h>
#include <boneflex/math/quat.h>
#include <boneflex/physics/distance_constraint_generation.h>
#include <boneflex/physics/mesh_reduction.h>
#include <boneflex/physics/reduced_soft_body_solver.h>
#include <boneflex/physics/shape_matching_cluster_generation.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iterator>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")

namespace
{
    using Microsoft::WRL::ComPtr;
    using namespace DirectX;

    constexpr float pi = 3.14159265358979323846f;
    constexpr float fixed_delta_time = 1.0f / 60.0f;

    struct render_vertex
    {
        XMFLOAT3 position;
        XMFLOAT3 normal;
    };

    struct constant_data
    {
        XMFLOAT4X4 world_view_projection;
        XMFLOAT4X4 world;
    };

    struct demo_state
    {
        HWND window = nullptr;
        UINT width = 1280;
        UINT height = 800;
        bool running = true;
        bool dragging = false;
        int selected_control_point = -1;
        POINT mouse_position{};

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        ComPtr<IDXGISwapChain> swap_chain;
        ComPtr<ID3D11RenderTargetView> render_target;
        ComPtr<ID3D11DepthStencilView> depth_view;
        ComPtr<ID3D11Buffer> vertex_buffer;
        ComPtr<ID3D11Buffer> index_buffer;
        ComPtr<ID3D11Buffer> constant_buffer;
        ComPtr<ID3D11VertexShader> vertex_shader;
        ComPtr<ID3D11PixelShader> pixel_shader;
        ComPtr<ID3D11InputLayout> input_layout;
        ComPtr<ID3D11RasterizerState> rasterizer_state;

        boneflex::blender_asset imported_asset;
        bool has_imported_asset = false;
        bool animation_enabled = true;
        bool gravity_enabled = true;
        float animation_time = 0.0f;
        std::size_t animated_bone_index = 0;
        boneflex::vec3 imported_normalization_center{};
        float imported_normalization_scale = 1.0f;
        std::vector<float> control_bone_weights;
        std::vector<bool> control_anchors;
        std::vector<boneflex::vec3> render_rest_positions;
        std::vector<std::uint32_t> render_indices;
        std::vector<render_vertex> render_vertices;
        std::vector<boneflex::vec3> control_accelerations;
        boneflex::reduced_mesh reduced;
        boneflex::vertex_embedding embedding;
        boneflex::reduced_soft_body_solver solver;
        std::vector<boneflex::vec3> deformed_positions;
        std::vector<boneflex::vec3> accumulated_normals;
        std::vector<std::vector<std::uint32_t>> render_neighbors;
        std::vector<boneflex::vec3> smoothing_positions;
        std::vector<boneflex::xpbd_distance_constraint> constraints;
        boneflex::shape_matching_clusters clusters;
        boneflex::vec3 drag_target{};
        XMMATRIX view = XMMatrixIdentity();
        XMMATRIX projection = XMMatrixIdentity();
    };

    demo_state* active_demo = nullptr;

    void create_uv_sphere(
        std::uint32_t longitude_segments,
        std::uint32_t latitude_segments,
        float radius,
        std::vector<boneflex::vec3>& positions,
        std::vector<std::uint32_t>& indices)
    {
        positions.clear();
        indices.clear();

        for (std::uint32_t y = 0; y <= latitude_segments; ++y)
        {
            const float v = static_cast<float>(y) / static_cast<float>(latitude_segments);
            const float latitude = v * pi;
            const float ring_radius = std::sin(latitude);
            const float py = std::cos(latitude);

            for (std::uint32_t x = 0; x <= longitude_segments; ++x)
            {
                const float u = static_cast<float>(x) / static_cast<float>(longitude_segments);
                const float longitude = u * 2.0f * pi;
                positions.emplace_back(
                    radius * ring_radius * std::cos(longitude),
                    radius * py,
                    radius * ring_radius * std::sin(longitude));
            }
        }

        for (std::uint32_t y = 0; y < latitude_segments; ++y)
        {
            for (std::uint32_t x = 0; x < longitude_segments; ++x)
            {
                const std::uint32_t a = y * (longitude_segments + 1) + x;
                const std::uint32_t b = a + longitude_segments + 1;
                indices.insert(indices.end(), { a, b, a + 1, a + 1, b, b + 1 });
            }
        }
    }

    bool initialize_simulation(demo_state& demo, const std::string& asset_path)
    {
        if (asset_path.empty())
        {
            create_uv_sphere(96, 48, 1.0f, demo.render_rest_positions, demo.render_indices);
        }
        else
        {
            std::string import_error;
            if (!boneflex::load_blender_asset(asset_path, demo.imported_asset, &import_error))
            {
                std::cerr << "Blender asset import failed: " << import_error << '\n';
                return false;
            }
            if (!demo.imported_asset.skin_current_pose(demo.render_rest_positions))
            {
                std::cerr << "Imported bind-pose skinning failed.\n";
                return false;
            }
            demo.render_indices = demo.imported_asset.triangle_indices;
            demo.has_imported_asset = true;

            boneflex::vec3 minimum = demo.render_rest_positions.front();
            boneflex::vec3 maximum = minimum;
            for (const boneflex::vec3& position : demo.render_rest_positions)
            {
                minimum.x = std::min(minimum.x, position.x);
                minimum.y = std::min(minimum.y, position.y);
                minimum.z = std::min(minimum.z, position.z);
                maximum.x = std::max(maximum.x, position.x);
                maximum.y = std::max(maximum.y, position.y);
                maximum.z = std::max(maximum.z, position.z);
            }

            const boneflex::vec3 center = (minimum + maximum) * 0.5f;
            const boneflex::vec3 extent = maximum - minimum;
            const float maximum_extent = std::max({ extent.x, extent.y, extent.z });
            if (!std::isfinite(maximum_extent) || maximum_extent <= 1.0e-6f)
            {
                std::cerr << "Imported mesh has invalid or degenerate bounds.\n";
                return false;
            }
            const float normalization_scale = 2.0f / maximum_extent;
            demo.imported_normalization_center = center;
            demo.imported_normalization_scale = normalization_scale;
            for (boneflex::vec3& position : demo.render_rest_positions)
            {
                position = (position - center) * normalization_scale;
            }
        }

        boneflex::mesh_reduction_options reduction_options;
        reduction_options.target_control_point_count = 1200;
        reduction_options.embedding_influence_count = 4;

        std::string error;
        if (!boneflex::generate_reduced_mesh(
            demo.render_rest_positions, demo.render_indices,
            reduction_options, demo.reduced, demo.embedding, &error))
        {
            std::cerr << "Mesh reduction failed: " << error << '\n';
            return false;
        }

        if (demo.has_imported_asset)
        {
            const std::size_t control_count = demo.reduced.control_point_count();
            const std::size_t bone_count = demo.imported_asset.rig.bone_count();
            demo.control_bone_weights.assign(control_count * bone_count, 0.0f);
            demo.control_anchors.assign(control_count, false);
            const auto& eo = demo.embedding.offsets();
            const auto& ei = demo.embedding.control_point_indices();
            const auto& ew = demo.embedding.weights();
            const auto& so = demo.imported_asset.weights.offsets();
            const auto& sb = demo.imported_asset.weights.bone_indices();
            const auto& sw = demo.imported_asset.weights.weights();
            for (std::size_t vertex = 0; vertex < demo.render_rest_positions.size(); ++vertex)
            {
                for (std::size_t e = eo[vertex]; e < eo[vertex + 1]; ++e)
                {
                    const std::size_t cp = ei[e];
                    for (std::size_t w = so[vertex]; w < so[vertex + 1]; ++w)
                        demo.control_bone_weights[cp * bone_count + sb[w]] += ew[e] * sw[w];
                }
            }
            for (std::size_t cp = 0; cp < control_count; ++cp)
            {
                float total = 0.0f, maximum = 0.0f;
                for (std::size_t b = 0; b < bone_count; ++b)
                {
                    const float weight = demo.control_bone_weights[cp * bone_count + b];
                    total += weight;
                    maximum = std::max(maximum, weight);
                }
                if (total > 1.0e-8f)
                {
                    for (std::size_t b = 0; b < bone_count; ++b)
                        demo.control_bone_weights[cp * bone_count + b] /= total;
                    demo.control_anchors[cp] = maximum / total >= 0.98f;
                }
            }
            const auto& bones = demo.imported_asset.rig.bones();
            for (std::size_t i = 0; i < bones.size(); ++i)
                if (bones[i].parent_index >= 0) { demo.animated_bone_index = i; break; }
        }

        boneflex::shape_matching_cluster_options cluster_options;
        cluster_options.cluster_size = 64;
        cluster_options.overlap_ratio = 0.5f;
        if (!boneflex::generate_shape_matching_clusters(
            demo.reduced, cluster_options, demo.clusters, &error))
        {
            std::cerr << "Cluster generation failed: " << error << '\n';
            return false;
        }

        boneflex::distance_constraint_generation_options constraint_options;
        constraint_options.compliance = 1.0e-5f;
        if (!boneflex::generate_distance_constraints(
            demo.reduced, constraint_options, demo.constraints, &error))
        {
            std::cerr << "Constraint generation failed: " << error << '\n';
            return false;
        }

        std::vector<boneflex::particle> particles(demo.reduced.control_point_count());
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            particles[i].position = demo.reduced.rest_positions()[i];
            particles[i].previous_position = particles[i].position;
            particles[i].inverse_mass = 1.0f;
        }

        boneflex::reduced_soft_body_options solver_options;
        solver_options.solver_iterations = 4;
        solver_options.shape_matching_stiffness = 0.15f;
        solver_options.max_deformation_ratio = 0.15f;
        solver_options.velocity_damping = 0.995f;
        solver_options.gravity = boneflex::vec3();

        if (!demo.solver.initialize(
            particles, demo.reduced.rest_positions(), demo.constraints,
            demo.clusters, solver_options))
        {
            std::cerr << "Soft-body solver initialization failed.\n";
            return false;
        }

        demo.control_accelerations.resize(particles.size());
        demo.deformed_positions = demo.render_rest_positions;
        demo.accumulated_normals.resize(demo.render_rest_positions.size());
        demo.render_vertices.resize(demo.render_rest_positions.size());
        demo.render_neighbors.resize(demo.render_rest_positions.size());
        demo.smoothing_positions.resize(demo.render_rest_positions.size());
        for (std::size_t i = 0; i < demo.render_indices.size(); i += 3)
        {
            const std::uint32_t a = demo.render_indices[i];
            const std::uint32_t b = demo.render_indices[i + 1];
            const std::uint32_t c = demo.render_indices[i + 2];
            demo.render_neighbors[a].insert(demo.render_neighbors[a].end(), { b, c });
            demo.render_neighbors[b].insert(demo.render_neighbors[b].end(), { a, c });
            demo.render_neighbors[c].insert(demo.render_neighbors[c].end(), { a, b });
        }
        for (auto& neighbors : demo.render_neighbors)
        {
            std::sort(neighbors.begin(), neighbors.end());
            neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
        }

        std::cout << "Render vertices: " << demo.render_rest_positions.size() << '\n'
            << "Control points: " << demo.reduced.control_point_count() << '\n'
            << "Reduced triangles: " << demo.reduced.triangle_count() << '\n'
            << "Shape matching clusters: " << demo.clusters.cluster_count() << '\n'
            << "XPBD constraints: " << demo.constraints.size() << '\n';
        if (demo.has_imported_asset)
        {
            const std::size_t anchor_count = static_cast<std::size_t>(
                std::count(demo.control_anchors.begin(), demo.control_anchors.end(), true));
            std::cout << "Kinematic anchors: " << anchor_count << '\n'
                << "Dynamic control points: " << (demo.control_anchors.size() - anchor_count) << '\n'
                << "Anchor threshold: 98% dominant bone weight\n"
                << "Shape matching stiffness: 0.15\n"
                << "Maximum deformation ratio: 0.15\n"
                << "Animated bone: " << demo.imported_asset.rig.bones()[demo.animated_bone_index].name << '\n';
        }
        return true;
    }

    bool create_render_targets(demo_state& demo)
    {
        ComPtr<ID3D11Texture2D> back_buffer;
        HRESULT result = demo.swap_chain->GetBuffer(0, IID_PPV_ARGS(&back_buffer));
        if (FAILED(result)) return false;

        result = demo.device->CreateRenderTargetView(back_buffer.Get(), nullptr, &demo.render_target);
        if (FAILED(result)) return false;

        D3D11_TEXTURE2D_DESC depth_desc{};
        depth_desc.Width = demo.width;
        depth_desc.Height = demo.height;
        depth_desc.MipLevels = 1;
        depth_desc.ArraySize = 1;
        depth_desc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        depth_desc.SampleDesc.Count = 1;
        depth_desc.Usage = D3D11_USAGE_DEFAULT;
        depth_desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

        ComPtr<ID3D11Texture2D> depth_texture;
        result = demo.device->CreateTexture2D(&depth_desc, nullptr, &depth_texture);
        if (FAILED(result)) return false;

        result = demo.device->CreateDepthStencilView(depth_texture.Get(), nullptr, &demo.depth_view);
        if (FAILED(result)) return false;

        demo.context->OMSetRenderTargets(1, demo.render_target.GetAddressOf(), demo.depth_view.Get());
        D3D11_VIEWPORT viewport{};
        viewport.Width = static_cast<float>(demo.width);
        viewport.Height = static_cast<float>(demo.height);
        viewport.MinDepth = 0.0f;
        viewport.MaxDepth = 1.0f;
        demo.context->RSSetViewports(1, &viewport);
        return true;
    }

    bool initialize_graphics(demo_state& demo)
    {
        DXGI_SWAP_CHAIN_DESC swap_desc{};
        swap_desc.BufferCount = 2;
        swap_desc.BufferDesc.Width = demo.width;
        swap_desc.BufferDesc.Height = demo.height;
        swap_desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        swap_desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swap_desc.OutputWindow = demo.window;
        swap_desc.SampleDesc.Count = 1;
        swap_desc.Windowed = TRUE;
        swap_desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

        D3D_FEATURE_LEVEL feature_level{};
        const D3D_FEATURE_LEVEL feature_levels[] = {
            D3D_FEATURE_LEVEL_11_1,
            D3D_FEATURE_LEVEL_11_0
        };

        HRESULT result = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
            feature_levels, static_cast<UINT>(std::size(feature_levels)),
            D3D11_SDK_VERSION, &swap_desc, &demo.swap_chain,
            &demo.device, &feature_level, &demo.context);
        if (FAILED(result))
        {
            const D3D_FEATURE_LEVEL fallback_levels[] = { D3D_FEATURE_LEVEL_11_0 };
            result = D3D11CreateDeviceAndSwapChain(
                nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
                fallback_levels, 1, D3D11_SDK_VERSION, &swap_desc,
                &demo.swap_chain, &demo.device, &feature_level, &demo.context);
        }
        if (FAILED(result)) return false;

        static constexpr char shader_source[] = R"(
cbuffer Scene : register(b0)
{
    float4x4 world_view_projection;
    float4x4 world;
};

struct VSInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
};

struct PSInput
{
    float4 position : SV_POSITION;
    float3 normal : NORMAL;
};

PSInput VSMain(VSInput input)
{
    PSInput output;
    output.position = mul(float4(input.position, 1.0f), world_view_projection);
    output.normal = normalize(mul(float4(input.normal, 0.0f), world).xyz);
    return output;
}

float4 PSMain(PSInput input) : SV_TARGET
{
    float3 normal = normalize(input.normal);
    float3 light_direction = normalize(float3(-0.4f, 0.7f, -0.5f));
    float diffuse = saturate(dot(normal, light_direction));
    float lighting = 0.22f + diffuse * 0.78f;
    float3 base_color = float3(0.16f, 0.62f, 0.88f);
    return float4(base_color * lighting, 1.0f);
}
)";

        ComPtr<ID3DBlob> vertex_blob;
        ComPtr<ID3DBlob> pixel_blob;
        ComPtr<ID3DBlob> errors;
        result = D3DCompile(shader_source, sizeof(shader_source) - 1, nullptr, nullptr, nullptr,
            "VSMain", "vs_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &vertex_blob, &errors);
        if (FAILED(result))
        {
            if (errors) std::cerr << static_cast<const char*>(errors->GetBufferPointer()) << '\n';
            return false;
        }
        result = D3DCompile(shader_source, sizeof(shader_source) - 1, nullptr, nullptr, nullptr,
            "PSMain", "ps_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pixel_blob, &errors);
        if (FAILED(result))
        {
            if (errors) std::cerr << static_cast<const char*>(errors->GetBufferPointer()) << '\n';
            return false;
        }

        if (FAILED(demo.device->CreateVertexShader(
            vertex_blob->GetBufferPointer(), vertex_blob->GetBufferSize(), nullptr, &demo.vertex_shader))
            || FAILED(demo.device->CreatePixelShader(
                pixel_blob->GetBufferPointer(), pixel_blob->GetBufferSize(), nullptr, &demo.pixel_shader)))
        {
            return false;
        }

        const D3D11_INPUT_ELEMENT_DESC elements[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 }
        };
        if (FAILED(demo.device->CreateInputLayout(
            elements, static_cast<UINT>(std::size(elements)),
            vertex_blob->GetBufferPointer(), vertex_blob->GetBufferSize(), &demo.input_layout)))
        {
            return false;
        }

        D3D11_BUFFER_DESC vertex_desc{};
        vertex_desc.ByteWidth = static_cast<UINT>(demo.render_vertices.size() * sizeof(render_vertex));
        vertex_desc.Usage = D3D11_USAGE_DYNAMIC;
        vertex_desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        vertex_desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        if (FAILED(demo.device->CreateBuffer(&vertex_desc, nullptr, &demo.vertex_buffer))) return false;

        D3D11_BUFFER_DESC index_desc{};
        index_desc.ByteWidth = static_cast<UINT>(demo.render_indices.size() * sizeof(std::uint32_t));
        index_desc.Usage = D3D11_USAGE_IMMUTABLE;
        index_desc.BindFlags = D3D11_BIND_INDEX_BUFFER;
        D3D11_SUBRESOURCE_DATA index_data{};
        index_data.pSysMem = demo.render_indices.data();
        if (FAILED(demo.device->CreateBuffer(&index_desc, &index_data, &demo.index_buffer))) return false;

        D3D11_BUFFER_DESC constant_desc{};
        constant_desc.ByteWidth = sizeof(constant_data);
        constant_desc.Usage = D3D11_USAGE_DEFAULT;
        constant_desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        if (FAILED(demo.device->CreateBuffer(&constant_desc, nullptr, &demo.constant_buffer))) return false;

        D3D11_RASTERIZER_DESC raster_desc{};
        raster_desc.FillMode = D3D11_FILL_SOLID;
        raster_desc.CullMode = D3D11_CULL_NONE;
        raster_desc.DepthClipEnable = TRUE;
        if (FAILED(demo.device->CreateRasterizerState(&raster_desc, &demo.rasterizer_state))) return false;

        demo.context->RSSetState(demo.rasterizer_state.Get());
        return create_render_targets(demo);
    }

    void update_camera(demo_state& demo)
    {
        demo.view = XMMatrixLookAtLH(
            XMVectorSet(0.0f, 0.0f, -3.5f, 1.0f),
            XMVectorZero(),
            XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
        demo.projection = XMMatrixPerspectiveFovLH(
            XMConvertToRadians(45.0f),
            static_cast<float>(demo.width) / static_cast<float>(demo.height),
            0.1f, 100.0f);
    }

    bool project_to_screen(const demo_state& demo, const boneflex::vec3& position, POINT& output)
    {
        const XMMATRIX world_view_projection = demo.view * demo.projection;
        const XMVECTOR clip = XMVector3TransformCoord(
            XMVectorSet(position.x, position.y, position.z, 1.0f), world_view_projection);
        XMFLOAT3 ndc{};
        XMStoreFloat3(&ndc, clip);
        if (ndc.z < 0.0f || ndc.z > 1.0f) return false;
        output.x = static_cast<LONG>((ndc.x * 0.5f + 0.5f) * demo.width);
        output.y = static_cast<LONG>((-ndc.y * 0.5f + 0.5f) * demo.height);
        return true;
    }

    boneflex::vec3 screen_to_world(const demo_state& demo, POINT point, float world_z)
    {
        constexpr float camera_z = -3.5f;
        const float view_depth = std::max(0.1f, world_z - camera_z);
        const float half_height = std::tan(XMConvertToRadians(45.0f) * 0.5f) * view_depth;
        const float half_width = half_height
            * static_cast<float>(demo.width) / static_cast<float>(demo.height);
        const float x = (static_cast<float>(point.x) / demo.width * 2.0f - 1.0f) * half_width;
        const float y = (1.0f - static_cast<float>(point.y) / demo.height * 2.0f) * half_height;
        return boneflex::vec3(x, y, world_z);
    }

    void begin_drag(demo_state& demo, POINT point)
    {
        // Control points are not rendered as markers, so allow selecting the
        // nearest point within a generous screen-space radius on the visible mesh.
        float nearest_distance_squared = 64.0f * 64.0f;
        int nearest_index = -1;
        const auto& particles = demo.solver.particles();
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            if (demo.has_imported_asset && demo.control_anchors[i]) continue;
            POINT screen{};
            if (!project_to_screen(demo, particles[i].position, screen)) continue;
            const float dx = static_cast<float>(screen.x - point.x);
            const float dy = static_cast<float>(screen.y - point.y);
            const float distance_squared = dx * dx + dy * dy;
            if (distance_squared < nearest_distance_squared)
            {
                nearest_distance_squared = distance_squared;
                nearest_index = static_cast<int>(i);
            }
        }
        demo.selected_control_point = nearest_index;
        demo.dragging = nearest_index >= 0;
        demo.drag_target = screen_to_world(demo, point, 0.0f);
    }

    void update_simulation(demo_state& demo)
    {
        if (demo.has_imported_asset)
        {
            auto& rig = demo.imported_asset.rig;
            rig.reset_pose();
            if (demo.animation_enabled)
            {
                demo.animation_time += fixed_delta_time;
                const float angle = std::sin(demo.animation_time * 1.6f) * 0.42f;
                boneflex::transform pose = rig.local_poses()[demo.animated_bone_index];
                pose.rotation = pose.rotation * boneflex::quat::from_axis_angle(
                    boneflex::vec3(0.0f, 0.0f, 1.0f), angle);
                rig.set_local_pose(demo.animated_bone_index, pose);
            }
            rig.update_global_pose();
            const auto palette = rig.build_skinning_palette();
            const std::size_t bone_count = rig.bone_count();
            auto& particles = demo.solver.particles();
            for (std::size_t cp = 0; cp < particles.size(); ++cp)
            {
                particles[cp].inverse_mass = demo.control_anchors[cp] ? 0.0f : 1.0f;
                if (!demo.control_anchors[cp]) continue;
                const boneflex::vec3 source = demo.reduced.rest_positions()[cp]
                    / demo.imported_normalization_scale + demo.imported_normalization_center;
                boneflex::vec3 skinned;
                for (std::size_t b = 0; b < bone_count; ++b)
                {
                    const float weight = demo.control_bone_weights[cp * bone_count + b];
                    if (weight > 0.0f) skinned += palette[b].transform_point(source) * weight;
                }
                const boneflex::vec3 target = (skinned - demo.imported_normalization_center)
                    * demo.imported_normalization_scale;
                particles[cp].position = target;
                particles[cp].previous_position = target;
                particles[cp].velocity = boneflex::vec3();
            }
        }

        auto& particles = demo.solver.particles();
        if (!demo.has_imported_asset)
            for (boneflex::particle& particle : particles) particle.inverse_mass = 1.0f;
        const boneflex::vec3 gravity = demo.gravity_enabled
            ? boneflex::vec3(0.0f, -9.81f, 0.0f)
            : boneflex::vec3();
        for (std::size_t i = 0; i < particles.size(); ++i)
        {
            const bool anchored = demo.has_imported_asset && demo.control_anchors[i];
            demo.control_accelerations[i] = anchored ? boneflex::vec3() : gravity;
        }
        if (demo.dragging && demo.selected_control_point >= 0
            && static_cast<std::size_t>(demo.selected_control_point) < particles.size())
        {
            const std::size_t selected = static_cast<std::size_t>(demo.selected_control_point);
            boneflex::particle& dragged = particles[selected];
            const boneflex::vec3 target = screen_to_world(
                demo, demo.mouse_position, dragged.position.z);

            // A temporary kinematic target is reliable with stiff shape matching:
            // acceleration-only dragging was almost entirely canceled by the solver.
            dragged.inverse_mass = 0.0f;
            dragged.position = target;
            dragged.previous_position = target;
            dragged.velocity = boneflex::vec3();
        }

        if (!demo.solver.step(fixed_delta_time, demo.control_accelerations))
        {
            std::cerr << "Soft-body simulation step failed.\n";
            demo.running = false;
            return;
        }

        std::vector<boneflex::vec3> control_positions;
        control_positions.reserve(demo.solver.particles().size());
        for (const boneflex::particle& particle : demo.solver.particles())
        {
            control_positions.push_back(particle.position);
        }
        if (!boneflex::reconstruct_render_vertices(control_positions, demo.embedding, demo.deformed_positions))
        {
            std::cerr << "Render mesh reconstruction failed.\n";
            demo.running = false;
            return;
        }

        // Taubin smoothing suppresses piecewise-linear artifacts from reduced-mesh
        // reconstruction while largely preserving the overall surface volume.
        constexpr float smooth_lambda = 0.35f;
        constexpr float smooth_mu = -0.36f;
        for (int pass = 0; pass < 2; ++pass)
        {
            const float coefficient = (pass == 0) ? smooth_lambda : smooth_mu;
            for (std::size_t i = 0; i < demo.deformed_positions.size(); ++i)
            {
                const auto& neighbors = demo.render_neighbors[i];
                if (neighbors.empty())
                {
                    demo.smoothing_positions[i] = demo.deformed_positions[i];
                    continue;
                }
                boneflex::vec3 average;
                for (std::uint32_t neighbor : neighbors)
                    average += demo.deformed_positions[neighbor];
                average *= 1.0f / static_cast<float>(neighbors.size());
                demo.smoothing_positions[i] = demo.deformed_positions[i]
                    + (average - demo.deformed_positions[i]) * coefficient;
            }
            demo.deformed_positions.swap(demo.smoothing_positions);
        }

        std::fill(demo.accumulated_normals.begin(), demo.accumulated_normals.end(), boneflex::vec3());
        for (std::size_t i = 0; i < demo.render_indices.size(); i += 3)
        {
            const std::uint32_t a = demo.render_indices[i];
            const std::uint32_t b = demo.render_indices[i + 1];
            const std::uint32_t c = demo.render_indices[i + 2];
            const boneflex::vec3 edge_a = demo.deformed_positions[b] - demo.deformed_positions[a];
            const boneflex::vec3 edge_b = demo.deformed_positions[c] - demo.deformed_positions[a];
            const boneflex::vec3 normal(
                edge_b.y * edge_a.z - edge_b.z * edge_a.y,
                edge_b.z * edge_a.x - edge_b.x * edge_a.z,
                edge_b.x * edge_a.y - edge_b.y * edge_a.x);
            demo.accumulated_normals[a] += normal;
            demo.accumulated_normals[b] += normal;
            demo.accumulated_normals[c] += normal;
        }

        for (std::size_t i = 0; i < demo.render_vertices.size(); ++i)
        {
            const boneflex::vec3 normal = demo.accumulated_normals[i].normalized();
            demo.render_vertices[i].position = XMFLOAT3(
                demo.deformed_positions[i].x,
                demo.deformed_positions[i].y,
                demo.deformed_positions[i].z);
            demo.render_vertices[i].normal = XMFLOAT3(normal.x, normal.y, normal.z);
        }

        D3D11_MAPPED_SUBRESOURCE mapped{};
        if (SUCCEEDED(demo.context->Map(demo.vertex_buffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
        {
            std::memcpy(mapped.pData, demo.render_vertices.data(),
                demo.render_vertices.size() * sizeof(render_vertex));
            demo.context->Unmap(demo.vertex_buffer.Get(), 0);
        }
    }

    void render(demo_state& demo)
    {
        const float clear_color[] = { 0.035f, 0.045f, 0.065f, 1.0f };
        demo.context->ClearRenderTargetView(demo.render_target.Get(), clear_color);
        demo.context->ClearDepthStencilView(demo.depth_view.Get(),
            D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

        const XMMATRIX world = XMMatrixIdentity();
        const XMMATRIX world_view_projection = world * demo.view * demo.projection;
        constant_data constants{};
        XMStoreFloat4x4(&constants.world_view_projection, XMMatrixTranspose(world_view_projection));
        XMStoreFloat4x4(&constants.world, XMMatrixTranspose(world));
        demo.context->UpdateSubresource(demo.constant_buffer.Get(), 0, nullptr, &constants, 0, 0);

        const UINT stride = sizeof(render_vertex);
        const UINT offset = 0;
        ID3D11Buffer* vertex_buffer = demo.vertex_buffer.Get();
        demo.context->IASetVertexBuffers(0, 1, &vertex_buffer, &stride, &offset);
        demo.context->IASetIndexBuffer(demo.index_buffer.Get(), DXGI_FORMAT_R32_UINT, 0);
        demo.context->IASetInputLayout(demo.input_layout.Get());
        demo.context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        demo.context->VSSetShader(demo.vertex_shader.Get(), nullptr, 0);
        demo.context->VSSetConstantBuffers(0, 1, demo.constant_buffer.GetAddressOf());
        demo.context->PSSetShader(demo.pixel_shader.Get(), nullptr, 0);
        demo.context->DrawIndexed(static_cast<UINT>(demo.render_indices.size()), 0, 0);
        demo.swap_chain->Present(1, 0);
    }

    void resize(demo_state& demo, UINT width, UINT height)
    {
        if (width == 0 || height == 0 || !demo.swap_chain) return;
        demo.width = width;
        demo.height = height;
        demo.context->OMSetRenderTargets(0, nullptr, nullptr);
        demo.render_target.Reset();
        demo.depth_view.Reset();
        if (SUCCEEDED(demo.swap_chain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, 0)))
        {
            create_render_targets(demo);
            update_camera(demo);
        }
    }

    LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
    {
        demo_state* demo = active_demo;
        if (demo == nullptr) return DefWindowProcW(window, message, wparam, lparam);

        switch (message)
        {
        case WM_SIZE:
            if (wparam != SIZE_MINIMIZED)
                resize(*demo, LOWORD(lparam), HIWORD(lparam));
            return 0;
        case WM_LBUTTONDOWN:
        {
            POINT point{ GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam) };
            demo->mouse_position = point;
            begin_drag(*demo, point);
            SetCapture(window);
            return 0;
        }
        case WM_MOUSEMOVE:
            demo->mouse_position = POINT{ GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam) };
            return 0;
        case WM_LBUTTONUP:
            demo->dragging = false;
            demo->selected_control_point = -1;
            ReleaseCapture();
            return 0;
        case WM_KEYDOWN:
            if (wparam == VK_ESCAPE) PostQuitMessage(0);
            if (wparam == 'A' && demo->has_imported_asset)
            {
                demo->animation_enabled = !demo->animation_enabled;
                std::cout << "Skeleton animation " << (demo->animation_enabled ? "enabled" : "paused") << '\n';
            }
            if (wparam == 'G')
            {
                demo->gravity_enabled = !demo->gravity_enabled;
                std::cout << "Gravity " << (demo->gravity_enabled ? "enabled" : "disabled") << '\n';
            }
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProcW(window, message, wparam, lparam);
        }
    }
}

int main(int argc, char** argv)
{
    if (argc > 2)
    {
        std::cerr << "Usage: boneflex_dx11_soft_body_demo [character.bflex]\n";
        return 1;
    }

    const std::string asset_path = (argc == 2) ? argv[1] : std::string();
    HINSTANCE instance = GetModuleHandleW(nullptr);
    demo_state demo;
    active_demo = &demo;

    WNDCLASSEXW window_class{};
    window_class.cbSize = sizeof(window_class);
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = window_proc;
    window_class.hInstance = instance;
    window_class.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    window_class.lpszClassName = L"BoneFlexDx11DemoWindow";
    if (!RegisterClassExW(&window_class)) return 1;

    RECT window_rect{ 0, 0, static_cast<LONG>(demo.width), static_cast<LONG>(demo.height) };
    AdjustWindowRect(&window_rect, WS_OVERLAPPEDWINDOW, FALSE);
    const wchar_t* window_title = asset_path.empty()
        ? L"BoneFlex - DirectX 11 Soft Body Demo"
        : L"BoneFlex - Blender Character Soft Body Demo";
    demo.window = CreateWindowExW(
        0, window_class.lpszClassName, window_title,
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
        window_rect.right - window_rect.left, window_rect.bottom - window_rect.top,
        nullptr, nullptr, instance, nullptr);
    if (!demo.window) return 1;

    if (!initialize_simulation(demo, asset_path) || !initialize_graphics(demo))
    {
        MessageBoxW(demo.window, L"Failed to initialize the BoneFlex demo. See the console for details.",
            L"BoneFlex", MB_ICONERROR | MB_OK);
        return 1;
    }

    update_camera(demo);
    ShowWindow(demo.window, SW_SHOWDEFAULT);
    update_simulation(demo);
    std::cout << (asset_path.empty()
        ? "Drag the sphere with the left mouse button. Press G to toggle gravity; press Esc to exit.\n"
        : "Imported Blender character with automatic skeleton anchors and gravity. Drag the surface; A toggles animation, G toggles gravity, Esc exits.\n");

    auto previous_time = std::chrono::steady_clock::now();
    float accumulator = 0.0f;
    MSG message{};
    while (demo.running)
    {
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
        {
            if (message.message == WM_QUIT)
            {
                demo.running = false;
                break;
            }
            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
        if (!demo.running) break;

        const auto current_time = std::chrono::steady_clock::now();
        const float frame_time = std::chrono::duration<float>(current_time - previous_time).count();
        previous_time = current_time;
        accumulator += std::min(frame_time, 0.1f);

        while (accumulator >= fixed_delta_time)
        {
            update_simulation(demo);
            accumulator -= fixed_delta_time;
        }
        render(demo);
    }

    active_demo = nullptr;
    return 0;
}
