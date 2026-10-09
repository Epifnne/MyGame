function(add_runtime_sample target_name)
	add_executable(${target_name} ${ARGN})
	target_link_libraries(${target_name} PRIVATE RuntimeLib)
	target_compile_features(${target_name} PRIVATE cxx_std_20)
	set_target_properties(${target_name} PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${MYGAME_OUTPUT_ROOT}
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${MYGAME_OUTPUT_ROOT}
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${MYGAME_OUTPUT_ROOT}
		RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${MYGAME_OUTPUT_ROOT}
		RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${MYGAME_OUTPUT_ROOT}
	)
endfunction()

function(embed_sample_shader target_name shader_file shader_symbol)
	set(shader_path "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${shader_file}")
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${shader_path}")
	file(READ "${shader_path}" SHADER_SOURCE)
	set(SHADER_SYMBOL "${shader_symbol}")
	configure_file("${CMAKE_SOURCE_DIR}/Sample/ShaderSource.h.in"
		"${CMAKE_CURRENT_BINARY_DIR}/${shader_file}.h" @ONLY)
	target_include_directories(${target_name} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()

add_subdirectory(MetalCube)
add_subdirectory(PhysicsCollision)
add_subdirectory(BoxStack)
add_subdirectory(FieldRainBenchmark)
add_subdirectory(JoltBoxFill)