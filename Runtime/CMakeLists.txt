find_package(OpenGL REQUIRED)

if(NOT COMMAND glad_add_library)
	message(FATAL_ERROR "glad_add_library is unavailable. Ensure ThirdParty/CMakeLists.txt is added before Runtime.")
endif()

glad_add_library(mygame_glad_gl33 STATIC REPRODUCIBLE API "gl:core=3.3")

file(GLOB_RECURSE RUNTIME_SOURCES CONFIGURE_DEPENDS
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(RuntimeLib STATIC ${RUNTIME_SOURCES})

target_include_directories(RuntimeLib
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${CMAKE_SOURCE_DIR}/ThirdParty/glm
	PRIVATE
		${CMAKE_SOURCE_DIR}/ThirdParty/stb
)

target_link_libraries(RuntimeLib
	PUBLIC
		SharedLib
		glfw
		mygame_glad_gl33
	PRIVATE
		OpenGL::GL
)

target_compile_definitions(RuntimeLib PRIVATE GLFW_INCLUDE_NONE)
target_compile_features(RuntimeLib PUBLIC cxx_std_17)
