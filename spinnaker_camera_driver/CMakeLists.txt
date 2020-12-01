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
project(spinnaker_camera_driver)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  cmake_minimum_required(VERSION 2.8.3)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  find_package(catkin REQUIRED)
  find_package(SPINNAKER)
  
  if(SPINNAKER_FOUND)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
    )

    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
      ${SPINNAKER_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME} SHARED
      src/camera_list_wrapper.cpp
      src/camera_settings.cpp
      src/camera_wrapper.cpp
      src/system_wrapper.cpp
    )

    target_link_libraries(${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${SPINNAKER_LIBRARIES}
    )

    install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
  else()
    message(WARNING "SPINNAKER SDK not found, so ${PROJECT_NAME} could not be built.")
    catkin_package()
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
  find_package(SPINNAKER)

  if(SPINNAKER_FOUND)
    ament_auto_find_build_dependencies()
    ament_auto_add_library(${PROJECT_NAME} SHARED
      "include/spinnaker_camera_driver/visibility_control.hpp"
      "include/spinnaker_camera_driver/system_wrapper.hpp"
      "include/spinnaker_camera_driver/camera_settings.hpp"
      "include/spinnaker_camera_driver/camera_wrapper.hpp"
      "include/spinnaker_camera_driver/camera_list_wrapper.hpp"
      "src/system_wrapper.cpp"
      "src/camera_settings.cpp"
      "src/camera_wrapper.cpp"
      "src/camera_list_wrapper.cpp")
    ## SPINNAKER is not handled by ament, so we need the manual steps below.
    target_include_directories(${PROJECT_NAME} PUBLIC ${SPINNAKER_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${SPINNAKER_LIBRARIES})
    # These are external libraries and include folders that we want to be visible to the packages
    # that depend on this one.
    ament_export_include_directories(${SPINNAKER_INCLUDE_DIRS})
    ament_export_libraries(${SPINNAKER_LIBRARIES})

    ## Testing
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()
  else()
    message(WARNING "SPINNAKER SDK not found, so ${PROJECT_NAME} could not be built.")
  endif()

  ament_auto_package()

endif()
