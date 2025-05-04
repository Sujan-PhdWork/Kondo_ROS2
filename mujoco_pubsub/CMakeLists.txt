cmake_minimum_required(VERSION 3.8)
project(mujoco_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set MuJoCo include and library paths (update these)
set(MUJOCO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Thirdparty/mujoco/include")
set(MUJOCO_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/Thirdparty/mujoco/lib")

# file(GLOB MUJOCO_LIBS "${MUJOCO_LIBRARY_DIR}/*.so")

# # Print out the libraries found

# # # VERBOSE=1 colcon build --event-handlers console_direct+  --packages-select mujoco_pubsub

# message(STATUS "MuJoCo shared libraries found:")
# foreach(lib ${MUJOCO_LIBS})
#     message(STATUS "  ${lib}")
# endforeach()

# Add the include directory
include_directories(${MUJOCO_INCLUDE_DIR})

# Add the library path
link_directories(${MUJOCO_LIBRARY_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(mujoco_node src/basic_publisher.cpp)
ament_target_dependencies(mujoco_node rclcpp std_msgs)
target_link_libraries(mujoco_node mujoco)

add_executable(visual_node src/basic_visual_publisher.cpp)
ament_target_dependencies(visual_node rclcpp std_msgs)
target_link_libraries(visual_node mujoco glfw)


# Embed library path in the binary to avoid LD_LIBRARY_PATH issues
set_target_properties(mujoco_node PROPERTIES
    BUILD_RPATH "${MUJOCO_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_LIBRARY_DIR}"
)

# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/media/sujan/Work_DIsk/one_week/mujoco_ws/src/mujoco_pubsub/Thirdparty/mujoco/lib

install(TARGETS
  mujoco_node
  visual_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
