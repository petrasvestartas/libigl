cmake_minimum_required(VERSION 3.14)
project(LibiglExample)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(ExternalProject)
include(FetchContent)

# Define source directories for external dependencies
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(EIGEN_SOURCE_DIR "${EXTERNAL_DIR}/eigen")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
# Enable parallel builds
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Create a custom target for all external dependencies
add_custom_target(external_downloads ALL)

# Download and configure Eigen
if(NOT EXISTS "${EIGEN_SOURCE_DIR}")
    message(STATUS "Downloading Eigen...")
    ExternalProject_Add(
        eigen_download
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        SOURCE_DIR "${EIGEN_SOURCE_DIR}"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
    )
    add_dependencies(external_downloads eigen_download)
endif()

# Fetch libigl
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG main
)
FetchContent_MakeAvailable(libigl)

# Add include directories
set(EIGEN_INCLUDE_DIR "${EIGEN_SOURCE_DIR}")

include_directories(
    ${EIGEN_INCLUDE_DIR}
)

# Add an example source file
set(EXAMPLE_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ray_mesh_intersection.cpp)

# Add the example executable
add_executable(${PROJECT_NAME} ${EXAMPLE_SOURCE_FILE})

# Ensure that all libraries are downloaded before compiling our project
add_dependencies(${PROJECT_NAME} external_downloads)

# Include library headers
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${EIGEN_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE igl::core)

# Link pthread library
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# Add a custom target for formatting (optional)
option(ENABLE_FORMATTING "Enable clang-format for code formatting" OFF)
if(ENABLE_FORMATTING)
    add_custom_target(format
        COMMAND clang-format -i ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        COMMENT "Formatting C++ files"
    )
    # Ensure that the format target runs before building the example
    add_dependencies(${PROJECT_NAME} format)
endif()

# Debug flags (platform specific)
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()
