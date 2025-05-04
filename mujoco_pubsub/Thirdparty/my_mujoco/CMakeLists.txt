cmake_minimum_required(VERSION 3.10)
project(MyMujocoProject)

set(CMAKE_CXX_STANDARD 17)

# Set MuJoCo include and library paths (update these)
set(MUJOCO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../abc/sujan_mujoco/include")
set(MUJOCO_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/../abc/sujan_mujoco/lib")


file(GLOB MUJOCO_LIBS "${MUJOCO_LIBRARY_DIR}/*.so")

# Print out the libraries found
message(STATUS "MuJoCo shared libraries found:")
foreach(lib ${MUJOCO_LIBS})
    message(STATUS "  ${lib}")
endforeach()
# Add the include directory
include_directories(${MUJOCO_INCLUDE_DIR})

# Add the library path
link_directories(${MUJOCO_LIBRARY_DIR})

# Source file already exists in src/
add_executable(main src/basic.cc)  # or replace with your real source file name
add_executable(main2 src/basic_visual.cc)  # or replace with your real source file name
# Link against mujoco libraries (change if your lib has a different name)
target_link_libraries(main mujoco glfw)
target_link_libraries(main2 mujoco glfw)

# If MuJoCo also needs other system libs like pthread or dl, add them:
#target_link_libraries(my_program pthread dl m)


