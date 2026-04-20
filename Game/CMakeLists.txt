file(GLOB_RECURSE GAME_SOURCES CONFIGURE_DEPENDS
	${CMAKE_CURRENT_SOURCE_DIR}/src/Core/*.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/Systems/*.cpp
)

add_library(GameLib STATIC ${GAME_SOURCES})

target_include_directories(GameLib
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(GameLib
	PUBLIC
		RuntimeLib
		SharedLib
)

target_compile_features(GameLib PUBLIC cxx_std_17)
