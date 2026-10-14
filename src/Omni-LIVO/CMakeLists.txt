cmake_minimum_required(VERSION 2.8.3)
project(omni_livo)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops")
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()


set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV REQUIRED
        COMPONENTS
        core
        highgui
        features2d
)
# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        pcl_ros
        tf
        tf2_ros
        livox_ros_driver
        message_generation
        eigen_conversions
        vikit_common
        vikit_ros
        cv_bridge
        image_transport
        rosbag
)
#find_package(teaserpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# OpenCV already found above with correct version

# Find Sophus and manually link to libSophus.so
FIND_PACKAGE(Sophus REQUIRED)
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")
# Make sure Boost is found
find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)


# Include directories for dependencies
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        include
)

# Define the catkin package
catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport
        DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)
# Add the main executable
add_executable(fastlivo_mapping src/main.cpp
)


# Link libraries to the executable
target_link_libraries(fastlivo_mapping
        laser_mapping
        vio
        lio
        pre
        imu_proc
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Sophus_LIBRARIES}  # Ensure linking to libSophus.so
        ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# The multi-sensor runtime shares Omni-LIVO's voxel planes and photometric warp,
# while owning a dynamic joint state instead of extending the legacy 19-D layout.
add_library(rig_core src/rig/core.cpp src/rig/config.cpp)
target_link_libraries(rig_core yaml-cpp)
if(OpenMP_CXX_FOUND)
  target_link_libraries(rig_core OpenMP::OpenMP_CXX)
endif()
add_library(rig_frontend src/rig/geometry.cpp src/rig/visual.cpp)
target_link_libraries(rig_frontend rig_core lio vio ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(rig_mapping src/rig/mapper.cpp src/rig/main.cpp)
target_link_libraries(rig_mapping rig_frontend rig_core lio vio laser_mapping imu_proc pre
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  add_executable(rig_core_test tests/rig_core_test.cpp)
  target_link_libraries(rig_core_test rig_core)
  add_test(NAME rig_core_test COMMAND rig_core_test)
  add_executable(rig_frontend_test tests/rig_frontend_test.cpp)
  target_link_libraries(rig_frontend_test rig_frontend lio vio laser_mapping imu_proc pre
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
  add_test(NAME rig_frontend_test COMMAND rig_frontend_test)
  add_test(NAME rig_converter_test COMMAND ${PYTHON_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_hbs_multi.py -v)
endif()
install(TARGETS rig_mapping rig_core rig_frontend fastlivo_mapping lio vio pre imu_proc laser_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch rviz_cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
