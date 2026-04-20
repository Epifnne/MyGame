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

target_compile_features(ECS_Test PRIVATE cxx_std_17)

gtest_discover_tests(ECS_Test)

add_executable(Graphics_Test
    Graphics/Graphics_BasicTest.cpp
)
set_test_output_dir(Graphics_Test)

target_link_libraries(Graphics_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(Graphics_Test PRIVATE cxx_std_17)

gtest_discover_tests(Graphics_Test)

add_executable(Physics_Test
    Physics/Physics_BasicTest.cpp
)
set_test_output_dir(Physics_Test)

target_link_libraries(Physics_Test
    PRIVATE
        RuntimeLib
        GTest::gtest_main
)

target_compile_features(Physics_Test PRIVATE cxx_std_17)

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

target_compile_features(Resource_Test PRIVATE cxx_std_17)

gtest_discover_tests(Resource_Test)
