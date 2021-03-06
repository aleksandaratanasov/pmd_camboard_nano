Overview
========

This package provides a [ROS][] driver for [PMD[vision]® CamBoard nano][PMD]
depth sensor plus a set of processing nodes for outlier removal, normal estimation and mesh generation from the data captured by the camboard.

The driver is packaged as a nodelet, therefore it may be directly merged inside
another ROS node to avoid unnecessary data copying. At the same time, it may be
started standalone or within a nodelet manager. The distance, depth (see
"Distance vs. depth images" section), amplitude, and point cloud data are
retrieved from the device and processed only if there are subscribers on the
corresponding topics.

The `pmd_camboard_nano.launch` script (inspired by the [openni_launch][] stack
in ROS) starts the driver nodelet along with the image rectification nodelets.

Each launch file for a processing node starts with `pmd_camboard_nano_cloud_` followed 
by the functionality it contains. Currently following processing nodes are included:

 * **Statistical outlier removal** - for removing a cloud's outliers using the SOR filter
 * **Normal estimation** - for calculating a cloud's normals (optional but encouraged: enable OpenMP)
 * **Surface smoothing** - for smoothing a cloud's surface (note: extensive testing pending)
 * **Mesh generators** - for generating a mesh from a cloud  
  * **Fast triangulation** - uses Greedy projection algorithm
  * **Poisson** - uses Poisson algorithm
  * **NURBS** - uses NURBS (warning: poor performance and incomplete launch file; the *on_nurbs* module is also currently unstable!)
 * **Iterative Closest Point** - for cloud registration using the ICP algorithm (warning: unstable!)
 
Each node has the option of writing its output to a file:
 
  * For point clouds - use binary compressed PCD files
  * For meshes - use STL files
  
Almost all launch files (with the exception of that for NURBS) provide a set of parameters for the functionality that the 
corresponding node provides allowing testing with various parameters without the need to recompile the code. In addition 
each node is almost completely encapsulated that is it provides a single functionality allowing a relatively flexible way 
of maintaining the code base. Refer to the `CMakeLists.txt` for further details.

**Important:** the package has low requirements on RAM however CPU performance is crucial. Especially the NURBS node exibits very 
poor performance and requires a lot of time to generate a mesh. It is adviced that it is done offline (using the created 
PCD files).

Installation
============

PMD SDK installation
--------------------

This package requires PMD SDK to be installed in the system. It will search for
the PMDSDK root folder locally in the package folder `<PACKAGE_FOLDER>/PMDSDK`,
globally in `/usr/local/pmd`, or systemwide defined by an environment variable
`PMDDIR`. You can also change the search folder by modifying the
`PMDSDK_ROOT_DIR` variable in `CMakeLists.txt` file.

You also need to copy the file `10-pmd-ubuntu.rules` provided with the SDK to
`/etc/udev/rules.d` to allow normal users to open the camera.

**EDIT:** With my CMakeLists.txt simply create the PMDSDK directory in the root
of the package with the subdirectories *include*, *lib* and *plugins*:

* **PMDSDK/include**: here you copy *pmddatadescription.h*, *pmdsdk2.h* and *pmdsdk2common.h*

* **PMDSDK/lib**: here you copy *libpmdaccess2.so*

* **PMDSDK/plugins**: here you copy *camboardnano.L64.pap* and *camboardnanoproc.L64.ppp*


Package installation
--------------------

Clone this repository into a local catkin workspace and simply call
`catkin_make` in the workspace folder. For details on building the processing nodes see the
`CMakeLists.txt`. For full build execute

    catkin_make -DENABLE_OPENMP=ON -DBUILD_WITH_NURBS=ON -DBUILD_WITH_FAST_TRIANGULATION=ON -DBUILD_WITH_POISSON=ON

ROS API
=======

pmd_camboard_nano::DriverNodelet
--------------------------------

### Published topics

* `distance/camera_info` (*sensor_msgs/CameraInfo*)  
  camera calibration and metadata (see "Camera calibration" section)

* `distance/image` (*sensor_msgs/Image*)  
  raw distances from the optical center of the device to scene points, contains
  `float` distances (mm)

* `depth/camera_info` (*sensor_msgs/CameraInfo*)  
  camera calibration and metadata (see "Camera calibration" section)

* `depth/image` (*sensor_msgs/Image*)  
  depths of scene points (distances along the camera optical axis) from the
  device, contains `float` depths in mm

* `amplitude/camera_info` (*sensor_msgs/CameraInfo*)  
  camera calibration and metadata (see "Camera calibration" section)

* `amplitude/image` (*sensor_msgs/Image*)  
  signal strengths of active illumination

* `points_unrectified` (*sensor_msgs/PointCloud2*)  
  3D point cloud generated by the camera driver

### Parameters

* `~device_serial` (default: "")  
  specifies which device to open, empty means any

* `~calibration_file`  
  path to the file with camera calibration data

* `~frame_id` (default: "/camera_optical_frame")  
  the tf frame of the camera

* `~open_camera_retry_period` (default: 3)  
  how often (seconds) to try to open camera during the startup

* `~update_rate` (default: 30)  
  how often (Hz) to download and publish new data from the camera

* `~flip_vertical` (default: true)  
  flip the output images/point clouds vertically, so that the first row is
  swapped with the last and so on

### Dynamically reconfigurable parameters

Use the [dynamic_reconfigure][] package to update these parameters in runtime:

* `~remove_invalid_pixels` (default: true)  
  replace invalid pixels in depth and amplitude images with NaNs

* `~integration_time` (default: 333)  
  integration time of the camera in us

* `~averaging_frames` (default: 0)  
  number of frames in sliding averaging window for distance data

* `~signal_strength_check` (default: true)  
  activate signal strength check

* `~signal_strength_threshold` (default: 200)  
  if the signal strength is below this threshold, the pixel is marked as invalid

* `~consistency_check` (default: true)  
  activate consistency check

* `~consistency_check_threshold` (default: 0.98)  
  if the consistency value of a pixel is below this threshold, it is marked as
  invalid

* `~bilateral_filter` (default: true)  
  enable/disable bilateral filtering of the depth images

* `~sigma_spatial` (default: 2.5)  
  spatial sigma parameter of the bilateral filter

* `~sigma_range` (default: 25)  
  range sigma parameter of the bilateral filter

* `~kernel_size` (default: 5)  
  kernel size parameter of the bilateral filter

* `~bilateral_filter_enhance_image` (default: false)  
  activate enhanced bilateral filtering (increases robustness against motion
  blur)

PCL
===
Installation
------------------
The mesh generation node is built with NURBS B-Spline support hence it requires the 
**on_nurbs** module which is not included in the upstream PCL 1.7.2 due its 
experimental nature. You have to build PCL from source if you want to generate 
meshes from the point clouds using NURBS. Following list provides the stepts to building PCL:

 1. Clone PCL from github. It is advisable to use as closer version to the one 
 provided as upstream package and used by ROS as possible. On my system (Debian
 Jessie) it is [PCL 1.7.2](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.7.2)

 2. Create a build folder inside the root folder of the source folder and execute
 `cmake ..` or `cmake-gui ..`. Make sure you set the “BUILD_surface_on_nurbs” flag.
 If your license permits, also enable “USE_UMFPACK” for sparse linear solving. This 
 requires SuiteSparse (libsuitesparse-dev in Debian-derived distros) which is faster, 
 allows more degrees of freedom (i.e. control points) and more data points. Leave the CMAKE_INSTALL_PATH
 to its default value `/usr/local`
 
 3. Build the sources using `make` inside the build folder
 4. Use `sudo make install` or `sudo checkinstall make install` to install the new version.
 I recommend the second since it allows you uninstall the custom build much easier using `dpkg`
 instead of hunting down all the files (the case when using `make install` only)
 5.(Optional but recommended) I strongly recommend removing the PCL from upstream and
 use only the custom build. It is also better to rebuild the PCL-related ROS packages. This part 
 at least on my machine made me cry so be advised! The recommendation comes from the fact that 
 mixing different versions of the same library might give you severe headaches later on. The 
 easiest way to do that is by removing the PCL-related ROS packages, download the required sources 
 in a catkin workspace (see the ROS wiki to hunt down all those packages), build and install these.
 
If NURBS is not enabled the mesh generator will use fast triangulation, which is faster but results
in a mesh of lower quality.
 
Misc
====

Camera calibration
------------------

By default the PMD plugin loads the calibration data from a file (provided with
the camera), which must be located within the working directory of the
application and have a name composed of the device serial number and *".dat"*
extension. If you are using the `pmd_camboard_nano.launch` file, the working
directory of the driver nodelet will be `~/.ros`. You therefore have to have a
copy of the calibration file there.

Alternatively, you can specify the location of the calibration data file as a
parameter of the nodelet (`~calibration_file`).

If the PMD plugin failed to load the calibration data, then the camera info
messages produced by the driver nodelet will be filled with the values that
*seem* to be "default" (see [this forum topic][calibration_forum_topic]).


Distance vs. depth images
-------------------------

The distance data provided by the PMD SDK driver is actually the distances from
the optical center of the camera to the scene points. Other cameras (e.g.
Microsoft Kinect) output depth maps that are composed of distances from the
cameras principal plane to the scene points along the optical axis. In other
words, their depth image consists of z-coordinates of the scene points in the
cameras coordinate frame.

This driver publishes both distance images (as output by the PMD SDK driver),
and "Kinect-style" depth images, computed by multiplying the distances with the
corresponding direction vectors.

Compatibility
-------------

This package was tested under Ubuntu Trusty x64 with ROS Indigo, Ubuntu Precise
x64 with ROS Fuerte, and Ubuntu Oneiric x64 with ROS Electric. The version of
PMD SDK is 1.3.2.

Known issues
------------

This package was tested on multiple PCs and generally worked fine, however on
one Lenovo laptop the following problems were observed:

* RViz crashed when trying to display the messages in the `/camera/points`
  topic.
  Workaround: set display style **NOT** to Points, e.g. to BillboardSpheres.
  **EDIT:** Works fine with my setup

* While adjusting the parameters with dynamic reconfigure GUI the driver nodelet
  freezed and sometimes even died.

* In case of a crash it is very possible for the camera device to become unavailable.
  This is due to the poorly written official driver, which utterly fails to handle such
  situations. If pmdClose() is not called (which happens in a case of a crash) the above
  described situation occurs.
  Workaround: detach and reattach the device, and then restart the pmd_camboard_nano_node

[ROS]: http://www.ros.org
[PMD]: http://www.pmdtec.com/products_services/reference_design.php
[openni_launch]: http://ros.org/wiki/openni_launch
[dynamic_reconfigure]: http://ros.org/wiki/dynamic_reconfigure
[calibration_forum_topic]: https://www.cayim.com/forum/index.php?/topic/33-intrinsics-and-calibration/#entry125
