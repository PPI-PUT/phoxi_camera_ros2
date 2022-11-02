# Copyright 2022 Amadeusz Szymko
# Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(phoxi_camera_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(phoxi_camera_msgs REQUIRED)

add_compile_options(-w)
add_compile_options(-std=c++17)
add_compile_options(-fpermissive)
add_compile_options(-pthread)
add_compile_options(-O2)

ament_auto_find_build_dependencies(REQUIRED
    ${${PROJECT_NAME}_BUILD_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

#include_directories(SYSTEM
#        ${PHOXI_INCLUDE_DIRS}
#        ${OpenCV_INCLUDE_DIRS}
#        )

set(PHOXI_CAMERA_ROS2_LIB_SRC
    src/phoxi_camera.cpp
    src/lifecycle_interface.cpp
)

set(PHOXI_CAMERA_ROS2_LIB_HEADERS
    include/phoxi_camera_ros2/phoxi_camera.hpp
    include/phoxi_camera_ros2/lifecycle_interface.hpp
    include/phoxi_camera_ros2/visibility_control.hpp
)

# generate library
ament_auto_add_library(phoxi_camera SHARED
    ${PHOXI_CAMERA_ROS2_LIB_SRC}
    ${PHOXI_CAMERA_ROS2_LIB_HEADERS}
)

set(PHOXI_CAMERA_ROS2_NODE_SRC
    src/phoxi_camera_node.cpp
)

set(PHOXI_CAMERA_ROS2_NODE_HEADERS
    include/phoxi_camera_ros2/phoxi_camera_node.hpp
)

# generate component node library
ament_auto_add_library(phoxi_camera_node SHARED
    ${PHOXI_CAMERA_ROS2_NODE_SRC}
    ${PHOXI_CAMERA_ROS2_NODE_HEADERS}
)

rclcpp_components_register_node(phoxi_camera_node
    PLUGIN "phoxi_camera::PhoxiCameraNode"
    EXECUTABLE phoxi_camera_node_exe
)

target_link_libraries(phoxi_camera
    ${PHOXI_LIBRARY}
    ${PCL_COMMON_LIBRARIES}
    ${OpenCV_LIBS}
    $<$<PLATFORM_ID:Linux>:rt>
)

target_include_directories(phoxi_camera PUBLIC
    ${PHOXI_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
    launch
    meshes
    param
    rviz
    urdf
)
