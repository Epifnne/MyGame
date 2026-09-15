cmake_minimum_required(VERSION 3.16)

# Top-level ThirdParty helper CMakeLists to expose subprojects used by the workspace.

# Disable examples/tests for dependency subprojects when possible to speed up build.
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Don't build GLFW examples")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Don't build GLFW tests")
set(GLFW_INSTALL OFF CACHE BOOL "Don't install GLFW")
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "Disable Wayland backend in CI/headless builds")

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/glfw ${CMAKE_BINARY_DIR}/thirdparty/glfw)

# Glad provides a helper cmake folder
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/glad/cmake ${CMAKE_BINARY_DIR}/thirdparty/glad)

set(JSON_BuildTests OFF CACHE BOOL "Don't build nlohmann_json tests" FORCE)
set(JSON_Install OFF CACHE BOOL "Don't install nlohmann_json" FORCE)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/nlohmann_json ${CMAKE_BINARY_DIR}/thirdparty/nlohmann_json)

# Mark that ThirdParty subprojects have been added for top-level builds
set(THIRD_PARTY_INCLUDED TRUE CACHE BOOL "ThirdParty subprojects added by workspace")

# Other third-party libraries can be added here later if needed
