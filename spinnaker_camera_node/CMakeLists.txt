# Copyright 2020 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

### Build the nodes
project(spinnaker_camera_node)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  cmake_minimum_required(VERSION 2.8.3)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    camera_info_manager
    sensor_msgs
    std_msgs
    nodelet
    spinnaker_camera_driver
  )
  find_package(SPINNAKER)
  
  if(SPINNAKER_FOUND)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
    )

    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
      ${SPINNAKER_INCLUDE_DIRS}
    )
    
    add_library(spinnaker_camera_nodelet SHARED
      src/nodelet.cpp
    )

    target_link_libraries(spinnaker_camera_nodelet
      ${catkin_LIBRARIES}
    )

    add_dependencies(spinnaker_camera_nodelet
      ${catkin_EXPORTED_TARGETS}
    )

    add_executable(${PROJECT_NAME}
      src/node.cpp
    )

    target_link_libraries(${PROJECT_NAME}
      spinnaker_camera_nodelet
      ${catkin_LIBRARIES}
    )

    add_dependencies(${PROJECT_NAME}
      ${catkin_EXPORTED_TARGETS}
    )

    install(TARGETS
      ${PROJECT_NAME}
      spinnaker_camera_nodelet
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    install(
      FILES
      spinnaker_camera_node.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
  
    install(
      DIRECTORY
        launch
        param
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  else()
    message(WARNING "SPINNAKER SDK not found, so ${PROJECT_NAME} could not be built.")
    find_package(catkin REQUIRED)
    catkin_package()

    install(
      DIRECTORY
        launch
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
  endif()
  

elseif(${ROS_VERSION} EQUAL 2)

  cmake_minimum_required(VERSION 3.5)
  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  ## dependencies
  find_package(ament_cmake_auto REQUIRED)

  # Even though Spinnaker is not required to build this package,
  # we must rely on it to only build this package if it exists.
  # The dependent package which does rely on Spinnaker (spinnaker_camera_driver)
  # must exist in the ament index to avoid an error on sourcing
  # the install space so we can't use that as the deciding factor.
  find_package(SPINNAKER)

  if(SPINNAKER_FOUND)
    ament_auto_find_build_dependencies()
    ament_auto_add_library(${PROJECT_NAME} SHARED
      "include/spinnaker_camera_node/visibility_control.hpp"
      "include/spinnaker_camera_node/rcl_node.hpp"
      "src/rcl_node.cpp")

    rclcpp_components_register_node(${PROJECT_NAME}
      PLUGIN "autoware::drivers::camera::SpinnakerCameraNode"
      EXECUTABLE ${PROJECT_NAME}_exe
    )

    ## Testing
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()
  else()
    message(WARNING "SPINNAKER SDK not found, so spinnaker_camera_node could not be built.")
  endif()

  ament_auto_package(
    INSTALL_TO_SHARE 
      param
      launch
  )

endif()
