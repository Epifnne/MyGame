include(GoogleTest)

set(MYGAME_TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../MyGame/tests)

function(set_test_output_dir target_name)
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${MYGAME_TEST_OUTPUT_DIR}
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${MYGAME_TEST_OUTPUT_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${MYGAME_TEST_OUTPUT_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${MYGAME_TEST_OUTPUT_DIR}
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${MYGAME_TEST_OUTPUT_DIR}
    )
endfunction()

set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK ON CACHE BOOL "" FORCE)

if(NOT TARGET gtest_main)
    add_subdirectory(
        ${CMAKE_SOURCE_DIR}/ThirdParty/googletest
        ${CMAKE_BINARY_DIR}/_deps/local-googletest-build
        EXCLUDE_FROM_ALL
    )
endif()

add_executable(ECS_Test
    ECS/RuntimeECS_SmokeTest.cpp
)
set_test_output_dir(ECS_Test)

target_link_libraries(ECS_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(ECS_Test PRIVATE cxx_std_20)

gtest_discover_tests(ECS_Test)

add_executable(CoreTypes_Test
    Common/CoreTypes_Test.cpp
    Common/JobSystem_Test.cpp
)
set_test_output_dir(CoreTypes_Test)

target_link_libraries(CoreTypes_Test
    PRIVATE
        RuntimeLib
        SharedLib
        GTest::gtest_main
)

target_compile_features(CoreTypes_Test PRIVATE cxx_std_20)

gtest_discover_tests(CoreTypes_Test)

add_executable(AI_Test
    AI/BehaviorTree_BasicTest.cpp
)
set_test_output_dir(AI_Test)

target_link_libraries(AI_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(AI_Test PRIVATE cxx_std_20)

gtest_discover_tests(AI_Test)

add_executable(Graphics_Test
    Graphics/Graphics_BasicTest.cpp
)
set_test_output_dir(Graphics_Test)

target_link_libraries(Graphics_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(Graphics_Test PRIVATE cxx_std_20)

gtest_discover_tests(Graphics_Test)

add_executable(Physics_Test
    Physics/Physics_BasicTest.cpp
    Physics/Physics_BroadPhaseTest.cpp
    Physics/Physics_ContactSolverTest.cpp
    Physics/Physics_MidphaseTest.cpp
    Physics/Physics_ParallelIslandSolverTest.cpp
    Physics/Physics_ParallelNarrowphaseTest.cpp
    Physics/Physics_PhysicsIslandTest.cpp
    Physics/Physics_RigidBodyTest.cpp
)
set_test_output_dir(Physics_Test)

target_link_libraries(Physics_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(Physics_Test PRIVATE cxx_std_20)

gtest_discover_tests(Physics_Test)

add_executable(Resource_Test
    Resource/Resource_BasicTest.cpp
)
set_test_output_dir(Resource_Test)

target_link_libraries(Resource_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(Resource_Test PRIVATE cxx_std_20)

gtest_discover_tests(Resource_Test)

add_executable(Network_Test
    Network/Network_BasicTest.cpp
)
set_test_output_dir(Network_Test)

target_link_libraries(Network_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(Network_Test PRIVATE cxx_std_20)

gtest_discover_tests(Network_Test)

add_executable(UI_Test
    UI/UI_BasicTest.cpp
)
set_test_output_dir(UI_Test)

target_link_libraries(UI_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(UI_Test PRIVATE cxx_std_20)

gtest_discover_tests(UI_Test)
