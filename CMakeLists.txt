cmake_minimum_required(VERSION 3.20)
project(MyGame VERSION 1.0.0 LANGUAGES CXX C)

include(CTest)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Set output directories
set(MYGAME_OUTPUT_ROOT ${CMAKE_BINARY_DIR}/../MyGame)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MYGAME_OUTPUT_ROOT})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MYGAME_OUTPUT_ROOT}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${MYGAME_OUTPUT_ROOT}/lib)

# Add subdirectories
add_subdirectory(ThirdParty)
add_subdirectory(Shared)
add_subdirectory(Runtime)
add_subdirectory(Game)
add_subdirectory(Tools)

if(BUILD_TESTING)
    add_subdirectory(Tests)
endif()

# Create main executable
if(EXISTS "${CMAKE_SOURCE_DIR}/Game/src/main.cpp")
    add_executable(MyGame Game/src/main.cpp)
    target_link_libraries(MyGame PRIVATE GameLib RuntimeLib SharedLib)

    if(EXISTS "${CMAKE_SOURCE_DIR}/Game/assets")
        add_custom_command(TARGET MyGame POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:MyGame>/assets"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_SOURCE_DIR}/Game/assets"
                "$<TARGET_FILE_DIR:MyGame>/assets"
            COMMENT "Copying Game assets to runtime output directory"
        )
    endif()
endif()

# Install target
if(TARGET MyGame)
    install(TARGETS MyGame
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    if(EXISTS "${CMAKE_SOURCE_DIR}/Game/assets")
        install(DIRECTORY "${CMAKE_SOURCE_DIR}/Game/assets/"
            DESTINATION ${CMAKE_INSTALL_BINDIR}/assets
        )
    endif()
endif()
