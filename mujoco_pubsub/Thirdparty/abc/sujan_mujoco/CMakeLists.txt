cmake_minimum_required(VERSION 3.10)
project(MyMujocoProject)

set(CMAKE_CXX_STANDARD 17)

# Set MuJoCo include and library paths (update these)
set(MUJOCO_INCLUDE_DIR "include")
set(MUJOCO_LIBRARY_DIR "lib")


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

# Link against mujoco libraries (change if your lib has a different name)
target_link_libraries(main mujoco)

# If MuJoCo also needs other system libs like pthread or dl, add them:
#target_link_libraries(my_program pthread dl m)
