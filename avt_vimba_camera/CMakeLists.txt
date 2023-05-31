cmake_minimum_required(VERSION 3.5)
project(avt_vimba_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(OpenCV 4 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rtx_msg_interface REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

ament_auto_find_build_dependencies()

# Get architecture
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(TargetArchitecture)
target_architecture(ARCH)

###########
## Build ##
###########

function(add_dependencies_and_linkings arg)
  if("${ARCH}" STREQUAL x86_64)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL armv7)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL armv8)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
    )
  else()
    message(FATAL_ERROR "[libvimba]: Architecture ${ARCH} not suported. Exiting...")
  endif()
endfunction(add_dependencies_and_linkings)

# Path to * project folder.
set(DARKNET_PATH ${PROJECT_SOURCE_DIR}/include/inference/darknet)

# Give a custom name to shared library which is provided by * when the one compiled.
set(DARKNET_LIB_NAME libdarknet)

# Link * library' include folder.
include_directories(${DARKNET_PATH}/include)

# Define * library' shared library.
add_library(${DARKNET_LIB_NAME} SHARED IMPORTED)

# Set a path to * library' shared library
set_property(TARGET ${DARKNET_LIB_NAME} PROPERTY IMPORTED_LOCATION "${DARKNET_PATH}/libdarknet.so")

# Nodes
ament_auto_add_library(mono_camera_node SHARED
  src/mono_camera_node.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
  include/cluster/cluster.cpp
  include/inference/inference.cpp
  include/pcan/ObjectDetectionsSender.cpp
)
ament_target_dependencies(mono_camera_node
  rtx_msg_interface
)
target_link_libraries(mono_camera_node pcanbasic)
target_link_libraries(mono_camera_node ${DARKNET_LIB_NAME})
add_dependencies_and_linkings(mono_camera_node)

ament_auto_add_library(trigger_node SHARED
  src/trigger_node.cpp
)
add_dependencies_and_linkings(trigger_node)

# Executables
ament_auto_add_executable(mono_camera_exec
  src/exec/mono_camera_exec.cpp
)

ament_auto_add_executable(trigger_exec
  src/exec/trigger_exec.cpp
)

#############
## Install ##
#############

if("${ARCH}" STREQUAL x86_64)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    DESTINATION lib
  )
elseif("${ARCH}" STREQUAL armv7)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    DESTINATION lib
  )
elseif("${ARCH}" STREQUAL armv8)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
    DESTINATION lib
  )
endif()

install(FILES ../README.md
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE
  calibrations
  config
  launch
)