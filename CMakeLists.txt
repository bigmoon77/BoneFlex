cmake_minimum_required(VERSION 3.20)

project(BoneFlex
    VERSION 0.1.0
    DESCRIPTION "GPU-accelerated bone-driven softbody simulation"
    LANGUAGES CXX
)

option(BONEFLEX_BUILD_TESTS "Build BoneFlex tests" ON)
option(BONEFLEX_BUILD_EXAMPLES "Build BoneFlex examples" ON)

add_library(boneflex
    src/animation/skeleton.cpp
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
endif()

if(BONEFLEX_BUILD_EXAMPLES)
    add_executable(boneflex_basic_example
        examples/basic_simulation.cpp
    )
    target_link_libraries(boneflex_basic_example PRIVATE BoneFlex::boneflex)
endif()
