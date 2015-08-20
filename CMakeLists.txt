cmake_minimum_required(VERSION 2.8.3)
project(pmd_camboard_nano)

#set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}") # Congrats! No C++11 :-/ http://stackoverflow.com/questions/32119437/weird-segmentation-fault-after-converting-a-ros-pointcloud2-message-to-pcl-point/32120491#32120491

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport nodelet dynamic_reconfigure tf)

generate_dynamic_reconfigure_options(cfg/PMD.cfg)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs image_transport nodelet dynamic_reconfigure tf pcl_conversions pcl_msgs pcl_ros
    DEPENDS boost_system PCL
)

# Set PMDSDK Requirements
set(PMDSDK_ROOT_DIR ${PROJECT_SOURCE_DIR}/PMDSDK) # Change this if the path is different
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
# Register PMDSDK
find_package(PMDSDK REQUIRED)

# Register Boost
find_package(Boost REQUIRED COMPONENTS system)
# Register PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#########
# BUILD #
#########
add_definitions(-DPMD_PLUGIN_DIR="${PMDSDK_PLUGIN_DIR}/")
include_directories(include ${catkin_INCLUDE_DIRS} ${PMDSDK_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# make sure configure headers are built before any node using them
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
message(${CATKIN_DEVEL_PREFIX})
add_library(${PROJECT_NAME}_nodelet src/driver_nodelet.cpp src/pmd_camboard_nano.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PMDSDK_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_node src/driver_node)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PMDSDK_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})


# The cloud handler receives point clouds either directly from the PMD publishing node (directly controlling
# the PMD device) or from the cloud relay. Once a point cloud message is received, it is further processed
add_executable(${PROJECT_NAME}_cloud_handler src/pmd_camboard_nano_cloud_handler)
target_link_libraries(${PROJECT_NAME}_cloud_handler ${catkin_LIBRARIES} boost_system pthread pcl_common pcl_io pcl_filters)

# TODO
# The cloud relay receives the output from the cloud publishing node (directly controlling the PMD device)
# and re-publish it ONLY when the user triggers it. This is basically used to reduce the amount of data
# and also to regulate which point clouds are passed onto the cloud handler
add_executable(${PROJECT_NAME}_cloud_relay src/pmd_camboard_nano_cloud_relay)
target_link_libraries(${PROJECT_NAME}_cloud_relay ${catkin_LIBRARIES} boost_system pthread pcl_common pcl_io pcl_filters)
