add_library(SharedLib STATIC
	src/BuildConfig.cpp
)

target_include_directories(SharedLib
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(SharedLib PUBLIC cxx_std_17)
