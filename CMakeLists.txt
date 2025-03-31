cmake_minimum_required(VERSION 3.14)
project(LibiglExample)

include(ExternalProject)

# Define source directories for external dependencies
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(EIGEN_SOURCE_DIR "${EXTERNAL_DIR}/eigen")
set(LIBIGL_SOURCE_DIR "${EXTERNAL_DIR}/libigl")
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

# Download and configure libigl
if(NOT EXISTS "${LIBIGL_SOURCE_DIR}")
    message(STATUS "Downloading libigl...")
    ExternalProject_Add(
        libigl_download
        URL https://github.com/libigl/libigl/archive/refs/heads/main.zip
        SOURCE_DIR "${LIBIGL_SOURCE_DIR}"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    add_dependencies(external_downloads libigl_download)
endif()

# Add include directories
set(EIGEN_INCLUDE_DIR "${EIGEN_SOURCE_DIR}")
set(LIBIGL_INCLUDE_DIR "${LIBIGL_SOURCE_DIR}/include")

include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${LIBIGL_INCLUDE_DIR}
)

# Add an example source file
set(EXAMPLE_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/quad_mesh_planarization.cpp)

# Add the example executable
add_executable(example ${EXAMPLE_SOURCE_FILE})

# Ensure that all libraries are downloaded before compiling our project
add_dependencies(example external_downloads)

# Include library headers
target_include_directories(example PRIVATE 
    ${EIGEN_INCLUDE_DIR}
    ${LIBIGL_INCLUDE_DIR}
)

# Link pthread library# Add a custom target for formatting
add_custom_target(format
    COMMAND clang-format -i ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    COMMENT "Formatting C++ files"
)

# Ensure that the format target runs before building the example
add_dependencies(example format)
find_package(Threads REQUIRED)
target_link_libraries(example PRIVATE Threads::Threads)

# Debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

