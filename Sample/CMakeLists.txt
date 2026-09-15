add_executable(PhysicsCollisionSample
	PhysicsCollisionSample.cpp
)

target_link_libraries(PhysicsCollisionSample
	PRIVATE
		RuntimeLib
)

target_compile_features(PhysicsCollisionSample PRIVATE cxx_std_20)

set_target_properties(PhysicsCollisionSample PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${MYGAME_OUTPUT_ROOT}
	RUNTIME_OUTPUT_DIRECTORY_DEBUG ${MYGAME_OUTPUT_ROOT}
	RUNTIME_OUTPUT_DIRECTORY_RELEASE ${MYGAME_OUTPUT_ROOT}
	RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${MYGAME_OUTPUT_ROOT}
	RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${MYGAME_OUTPUT_ROOT}
)