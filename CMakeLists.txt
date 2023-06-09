cmake_minimum_required(VERSION 3.10)

# set the project name
project(Nexus)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# Set up libraries

add_library(nexus STATIC
    ./src/nexus/Particle.h
    ./src/nexus/Constraint.h
    ./src/nexus/Constraint.cpp
    ./src/nexus/PBDSolver.h
    ./src/nexus/PBDSolver.cpp
    ./src/nexus/NexusObject.h
    ./src/nexus/NexusObject.cpp
    ./src/nexus/NexusCloth.h
    ./src/nexus/NexusCloth.cpp
    ./src/nexus/NexusRigidBody.h
    ./src/nexus/NexusRigidBody.cpp
    ./src/nexus/defines.h
    ./src/nexus/SpatialHash.h
    ./src/nexus/SpatialHash.cpp
    ./src/nexus/Helper.h
    ./src/nexus/Helper.cpp
)

target_include_directories(nexus PUBLIC
    ./3rdparty/glm
    ./3rdparty/eigen-3.2.8
    ./3rdparty/voxelizer
    ./src/nexus
)

# Set up executables/viewers
# Find OpenGL
find_package(OpenGL REQUIRED)

# Add GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(./3rdparty/glfw-3.3.2)

# Add GLAD
add_subdirectory(./3rdparty/glad)

# Add ImGUI
add_subdirectory(./3rdparty/imgui-1.77)

# Add tinyobj
add_library(tinyobj STATIC
    ./3rdparty/tinyobj/tiny_obj_loader.h
    ./3rdparty/tinyobj/tiny_obj_loader.cc
)

add_executable(ParticleViewer
    ./src/viewer/particleMain.cpp
    ./src/viewer/particleViewer.h
    ./src/viewer/particleViewer.cpp
    ./src/viewer/camera.cpp
    ./src/viewer/camera.h
    ./src/viewer/objmodel.h
    ./src/viewer/objmodel.cpp
    ./src/viewer/shader.h
    ./src/viewer/viewer.cpp
    ./src/viewer/viewer.h
    ./src/viewer/drawable.h
    ./src/viewer/drawable.cpp
)

target_include_directories(ParticleViewer PUBLIC 
    ./src/viewer
    ./src/animation
    ./src/nexus
    ./3rdparty/imgui-1.77
    ./3rdparty/glm
    ./3rdparty/tinyobj
    ./3rdparty/voxelizer
)

target_link_libraries(ParticleViewer PUBLIC glad glfw imgui tinyobj nexus)