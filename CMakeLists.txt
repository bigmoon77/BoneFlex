cmake_minimum_required(VERSION 3.20)

project(BoneFlex
    VERSION 0.1.0
    DESCRIPTION "GPU-accelerated bone-driven softbody simulation"
    LANGUAGES CXX
)

option(BONEFLEX_BUILD_TESTS "Build BoneFlex tests" ON)
option(BONEFLEX_BUILD_EXAMPLES "Build BoneFlex examples" ON)
option(BONEFLEX_BUILD_BENCHMARKS "Build BoneFlex performance benchmarks" OFF)
option(BONEFLEX_BUILD_DX11_DEMO "Build the Windows DirectX 11 interactive demo" OFF)

add_library(boneflex
    src/io/blender_asset.cpp
    src/animation/skeleton.cpp
    src/animation/skinning.cpp
    src/animation/skinning_weights.cpp
    src/physics/reduced_mesh.cpp
    src/physics/mesh_reduction.cpp
    src/physics/shape_matching.cpp
    src/physics/shape_matching_cluster_generation.cpp
    src/physics/reduced_soft_body_solver.cpp
    src/physics/distance_constraint_generation.cpp
    src/physics/xpbd_distance_constraint.cpp
)

add_library(BoneFlex::boneflex ALIAS boneflex)

target_compile_features(boneflex PUBLIC cxx_std_20)

target_include_directories(boneflex
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(MSVC)
    target_compile_options(boneflex PRIVATE /W4 /permissive-)
else()
    target_compile_options(boneflex PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BONEFLEX_BUILD_TESTS)
    enable_testing()
    add_executable(boneflex_xpbd_tests
        tests/test_xpbd_distance_constraint.cpp
    )
    target_link_libraries(boneflex_xpbd_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.XPBDDistanceConstraint COMMAND boneflex_xpbd_tests)

    add_executable(boneflex_skeleton_tests
        tests/test_skeleton.cpp
    )
    target_link_libraries(boneflex_skeleton_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.Skeleton COMMAND boneflex_skeleton_tests)

    add_executable(boneflex_skinning_tests
        tests/test_skinning.cpp
    )
    target_link_libraries(boneflex_skinning_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.Skinning COMMAND boneflex_skinning_tests)

    add_executable(boneflex_shape_matching_tests
        tests/test_shape_matching.cpp
    )
    target_link_libraries(boneflex_shape_matching_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.ShapeMatching COMMAND boneflex_shape_matching_tests)

    add_executable(boneflex_shape_matching_deformation_tests
        tests/test_shape_matching_deformation.cpp
    )
    target_link_libraries(boneflex_shape_matching_deformation_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.ShapeMatchingDeformation COMMAND boneflex_shape_matching_deformation_tests)

    add_executable(boneflex_shape_matching_cluster_tests
        tests/test_shape_matching_cluster_generation.cpp
    )
    target_link_libraries(boneflex_shape_matching_cluster_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.ShapeMatchingClusterGeneration COMMAND boneflex_shape_matching_cluster_tests)

    add_executable(boneflex_mesh_reduction_tests
        tests/test_mesh_reduction.cpp
    )
    target_link_libraries(boneflex_mesh_reduction_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.MeshReduction COMMAND boneflex_mesh_reduction_tests)

    add_executable(boneflex_reduced_soft_body_solver_tests
        tests/test_reduced_soft_body_solver.cpp
    )
    target_link_libraries(boneflex_reduced_soft_body_solver_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.ReducedSoftBodySolver COMMAND boneflex_reduced_soft_body_solver_tests)

    add_executable(boneflex_distance_constraint_generation_tests
        tests/test_distance_constraint_generation.cpp
    )
    target_link_libraries(boneflex_distance_constraint_generation_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.DistanceConstraintGeneration COMMAND boneflex_distance_constraint_generation_tests)

    add_executable(boneflex_blender_asset_tests
        tests/test_blender_asset.cpp
    )
    target_link_libraries(boneflex_blender_asset_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.BlenderAsset COMMAND boneflex_blender_asset_tests)

    add_executable(boneflex_reduced_mesh_tests
        tests/test_reduced_mesh.cpp
    )
    target_link_libraries(boneflex_reduced_mesh_tests PRIVATE BoneFlex::boneflex)
    add_test(NAME BoneFlex.ReducedMesh COMMAND boneflex_reduced_mesh_tests)
endif()

if(BONEFLEX_BUILD_EXAMPLES)
    add_executable(boneflex_basic_example
        examples/basic_simulation.cpp
    )
    target_link_libraries(boneflex_basic_example PRIVATE BoneFlex::boneflex)
endif()

if(BONEFLEX_BUILD_EXAMPLES)
    add_executable(boneflex_blender_asset_demo
        demos/blender_asset_import_demo.cpp
    )
    target_link_libraries(boneflex_blender_asset_demo PRIVATE BoneFlex::boneflex)
endif()

if(BONEFLEX_BUILD_BENCHMARKS)
    add_executable(boneflex_mesh_reduction_benchmark
        benchmarks/mesh_reduction_benchmark.cpp
    )
    target_link_libraries(boneflex_mesh_reduction_benchmark PRIVATE BoneFlex::boneflex)

    add_executable(boneflex_shape_matching_benchmark
        benchmarks/shape_matching_benchmark.cpp
    )
    target_link_libraries(boneflex_shape_matching_benchmark PRIVATE BoneFlex::boneflex)

    add_executable(boneflex_reduced_soft_body_benchmark
        benchmarks/reduced_soft_body_benchmark.cpp
    )
    target_link_libraries(boneflex_reduced_soft_body_benchmark PRIVATE BoneFlex::boneflex)

    add_executable(boneflex_reduced_soft_body_parameter_sweep
        benchmarks/reduced_soft_body_parameter_sweep.cpp
    )
    target_link_libraries(boneflex_reduced_soft_body_parameter_sweep PRIVATE BoneFlex::boneflex)
endif()

if(BONEFLEX_BUILD_DX11_DEMO)
    if(NOT WIN32)
        message(FATAL_ERROR "BONEFLEX_BUILD_DX11_DEMO requires Windows.")
    endif()

    add_executable(boneflex_dx11_soft_body_demo
        demos/dx11_soft_body_demo.cpp
    )
    add_custom_command(TARGET boneflex_dx11_soft_body_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/blender/import_blend_to_bflex.py
            $<TARGET_FILE_DIR:boneflex_dx11_soft_body_demo>/import_blend_to_bflex.py
    )
    target_link_libraries(boneflex_dx11_soft_body_demo
        PRIVATE
            BoneFlex::boneflex
            d3d11
            d3dcompiler
    )
endif()
