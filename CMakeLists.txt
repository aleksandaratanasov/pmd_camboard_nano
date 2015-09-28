cmake_minimum_required(VERSION 2.8.3)
project(pmd_camboard_nano)

# TODO Add dynamic reconfiguration for the processing nodes too
# TODO Move all class definitions to headers and the implementions to source files - create shared library (one per processing node or all together)

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
find_package(PCL 1.7 REQUIRED COMPONENTS io common filters features kdtree surface)
# Eigen
find_package(cmake_modules REQUIRED) # Required for Eigen
find_package(Eigen REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(include ${Eigen_INCLUDE_DIRS})
add_definitions(-DPMD_PLUGIN_DIR="${PMDSDK_PLUGIN_DIR}/")
include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PMDSDK_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

####################################################### BUILD #######################################################
# Make sure configure headers are built before any node using them
message("Setting up binaries for driver node and nodlet")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC") # Position Independent Code is very useful when multiple binaries use the same library.
                                                # This makes sure that the library is loaded once in memory and all binaries use it
add_library(${PROJECT_NAME}_nodelet src/driver_nodelet.cpp src/pmd_camboard_nano.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PMDSDK_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_node src/driver_node)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PMDSDK_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Following nodes are part of the processing pipeline. Each node takes a point cloud message,
# processes it and then publishes the results to the ROS network. This allows a very dynamic
# coupling between the processess working on the data from the PMD device and allows testing
# various combinations in order to achieve as good results as possible

# A cloud processing node receives point clouds either directly from the PMD publishing node (directly controlling
# the PMD device) or from the cloud relay. Once a point cloud message is received, it is further processed
message("Setting up binary for statistical outlier removal node")
add_executable(${PROJECT_NAME}_cloud_statistical_outlier_removal src/pmd_camboard_nano_cloud_statistical_outlier_removal)
target_link_libraries(${PROJECT_NAME}_cloud_statistical_outlier_removal ${PCL_LIBRARIES}) #<---UNTIL LINKING ORDER IS RESOLVED USE ALL PCL LIBS # pcl_common pcl_io pcl_filters pcl_surface pcl_kdtree)
target_link_libraries(${PROJECT_NAME}_cloud_statistical_outlier_removal ${catkin_LIBRARIES} boost_system pthread)

# Processing node: normal estimation
# Input: cloud
# Output: cloud with normals
message("Setting up binary for normal estimation")
option(ENABLE_OPENMP "Enable OpenMP for estimating the normals of a point cloud" OFF)
if(ENABLE_OPENMP)
  if(CMAKE_COMPILER_IS_GNUCXX)
    # Add this to a more global scope if other parts of the source code can use OpenMP and not only the normals estimation
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
    configure_file(config.h.in ${CMAKE_SOURCE_DIR}/include/config.h)
    message("OpenMP enabled")
  else()
    message("Non-GCC compiler detected. Add routine for your compiler in CMakeLists.txt to check for OpenMP support.")
    configure_file(config_empty.h.in ${CMAKE_SOURCE_DIR}/include/config.h)
  endif()
else()
  configure_file(config_empty.h.in ${CMAKE_SOURCE_DIR}/include/config.h)
  message("OpenMP disabled")
endif()
add_executable(${PROJECT_NAME}_cloud_normal_estimation src/pmd_camboard_nano_cloud_normal_estimation)
target_link_libraries(${PROJECT_NAME}_cloud_normal_estimation ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cloud_normal_estimation ${catkin_LIBRARIES} boost_system pthread)

# Processing node: iterative closest point
# Input: cloud
# Output: cloud (merged sequence of input clouds)
message("Setting up binary for iterative closest point")
add_executable(${PROJECT_NAME}_cloud_icp src/pmd_camboard_nano_cloud_icp.cpp)
target_link_libraries(${PROJECT_NAME}_cloud_icp ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cloud_icp ${catkin_LIBRARIES} boost_system pthread)

# Processing node: surface smoothing
# Input: cloud, normals
# Output: cloud
message("Setting up binary for surface smoothing and normal estimation node")
add_executable(${PROJECT_NAME}_cloud_surface_smoothing_normal_estimation src/pmd_camboard_nano_cloud_surface_smoothing_normal_estimation)
target_link_libraries(${PROJECT_NAME}_cloud_surface_smoothing_normal_estimation ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cloud_surface_smoothing_normal_estimation ${catkin_LIBRARIES} boost_system pthread)

# TODO one mesh generation procedure per node
option(BUILD_WITH_NURBS "Build mesh generator with NURBS B-spline fitting (requires on_nurbs module)" OFF)
option(BUILD_WITH_FAST_TRIANGULATION "Build mesh generator with fast triangulation" ON)
option(BUILD_WITH_POISSON "Build mesh generator with Poisson surface reconstruction" ON)

# Processing node: NURBS B-spline fitting (mesh generator)
# Due to the pretty tedious way of including a custom build PCL version alongside the upstream one that is used by
# ROS by default and the experimental nature of the on_nurbs module building the NURBS node is optional
if(BUILD_WITH_NURBS)
  message("Setting up binary for mesh generator node with NURBS B-spline fitting")
  message("Warning: the on_nurbs module of PCL is experimental")
  add_executable(${PROJECT_NAME}_cloud_mesh_generator_nurbs src/pmd_camboard_nano_cloud_mesh_generator_nurbs)
  target_link_libraries(${PROJECT_NAME}_cloud_mesh_generator_nurbs ${PCL_LIBRARIES})
  target_link_libraries(${PROJECT_NAME}_cloud_mesh_generator_nurbs ${catkin_LIBRARIES} boost_system pthread)
endif()

# Processing node: fast triangulation (mesh generator)
if(BUILD_WITH_FAST_TRIANGULATION)
  message("Setting up binary for mesh generator node with fast triangulation")
  add_executable(${PROJECT_NAME}_cloud_mesh_generator_fast_triangulation src/pmd_camboard_nano_cloud_mesh_generator_fast_triangulation)
  target_link_libraries(${PROJECT_NAME}_cloud_mesh_generator_fast_triangulation ${PCL_LIBRARIES})
  target_link_libraries(${PROJECT_NAME}_cloud_mesh_generator_fast_triangulation ${catkin_LIBRARIES} boost_system pthread)
endif()

# Processing node: Poisson surface reconstruction (mesh generator)
if(BUILD_WITH_POISSON)
  message("Setting up binary for mesh generator node with Poisson surface reconstruction")
  message("Warning: creates water-tight meshes")
  add_executable(${PROJECT_NAME}_cloud_mesh_generator_poisson src/pmd_camboard_nano_cloud_mesh_generator_poisson)
  target_link_libraries(${PROJECT_NAME}_cloud_mesh_generator_poisson ${PCL_LIBRARIES})
  target_link_libraries(${PROJECT_NAME}_cloud_mesh_generator_poisson ${catkin_LIBRARIES} boost_system pthread)
endif()
