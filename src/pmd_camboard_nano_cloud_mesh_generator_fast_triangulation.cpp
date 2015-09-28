/******************************************************************************
 * Copyright (c) 2015 Aleksandar Vladimirov Atanasov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/
// ROS
// ROS - Misc
#include <ros/ros.h>
// ROS - Publishing
//#include <ros/publisher.h>
#include <pcl_ros/publisher.h>

// PCL
// PCL - Misc
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>  // For writing mesh to STL file
//#include <pcl/io/ply_io.h>    // For writing mesh to PLY file
#include <pcl/io/vtk_io.h>      // For writing mesh to VTK file
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL - Mesh generation
// Use fast triangulation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

// Misc
#include <sstream>
#include <string>

#define degToRad(x) (M_PI*x/180)

class CloudProcessingNodeMGFT
{
  typedef pcl::PointXYZ Point;
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;
//  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;

private:
  sensor_msgs::PointCloud2 cloud;

  // For writing to files
  u_int64_t fileIdx;
  std::ostringstream ss;
  bool toggleWritingToFile;
  double searchRadius;
  double mu;
  int maxNN;
  double maxSurfaceAngle;
  double minAngle;
  double maxAngle;
  bool normalConsistency;

public:
  CloudProcessingNodeMGFT(std::string topicIn/*, std::string topicOut*/)
    : fileIdx(0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeMGFT::subCallback, this);
//    pub.advertise(nh, topicOut, 1);
  }

  ~CloudProcessingNodeMGFT()
  {
    sub.shutdown();
//    pub.shutdown();
  }

  void setWritingToFile(bool _toggle) { toggleWritingToFile = _toggle; }

  void setSearchRadius(double _searchRadius) { searchRadius = _searchRadius; }

  void setMu(double _mu) { mu = _mu; }

  void setMaxNN(int _maxNN) { maxNN = _maxNN; }

  void setMaxSurfaceAngle(double maxSurfaceAngle_radians) { maxSurfaceAngle = maxSurfaceAngle_radians; }

  void setMinAngle(double minAngle_radians) { minAngle = minAngle_radians; }

  void setMaxAngle(double maxAngle_radians) { maxAngle = maxAngle_radians; }

  void setNormalConsistency(bool _normalConsistency) { normalConsistency = _normalConsistency; }

  void subCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if(msg->data.empty())
    {
      ROS_WARN("Received an empty cloud message. Skipping further processing");
      return;
    }

    // Convert ROS message to PCL-compatible data structure
    ROS_INFO_STREAM("Received a cloud message with " << msg->height * msg->width << " points");
    ROS_INFO("Converting ROS cloud message to PCL compatible data structure");
    pcl::PointCloud<pcl::PointNormal> pclCloud;
    pcl::fromROSMsg(*msg, pclCloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr p_with_normals(new pcl::PointCloud<pcl::PointNormal>(pclCloud));

    // Fast triangulation
    // Source: http://pointclouds.org/documentation/tutorials/greedy_projection.php
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(p_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(searchRadius);

    // Set typical values for the parameters
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors(maxNN);
    gp3.setMaximumSurfaceAngle(maxSurfaceAngle); // 45 degrees
    gp3.setMinimumAngle(minAngle); // 10 degrees
    gp3.setMaximumAngle(maxAngle); // 120 degrees
    gp3.setNormalConsistency(normalConsistency);

    // Get result
    gp3.setInputCloud(p_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*mesh);

    ROS_INFO_STREAM("Mesh number of polygons: " << mesh->polygons.size());

    // Additional vertex information
    //std::vector<int> parts = gp3.getPartIDs();
    //std::vector<int> states = gp3.getPointStates();

    if(toggleWritingToFile)
    {
      //std::string path = "/home/redbaron/catkin_ws/src/pmd_camboard_nano/launch/";
      std::string path = "/home/redbaron/catkin_ws/src/pmd_camboard_nano/samples/temp/";
      ss << path << "cloud_mesh_generator_fast_triangulation_" << fileIdx << ".stl";
      pcl::io::savePolygonFileSTL(ss.str(), *mesh);
      ROS_INFO_STREAM("Writing to file \"" << ss.str() << "\"");
      fileIdx++;
      ss.str("");
    }

//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg(*p, output);
//    pub.publish(output);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_mesh_generator_fast_triangulation");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_ne";
//  std::string topicOut = "points_mg";
  bool toggleWriteToFile;
  double searchRadius;
  double mu;
  int maxNN;
  double maxSurfaceAngle;
  double minAngle;
  double maxAngle;
  bool normalConsistency;

  nh.param("write_to_file", toggleWriteToFile, false);
  nh.param("searchRadius", searchRadius, 0.025);
  nh.param("mu", mu, 2.5);
  nh.param("maxNN", maxNN, 100);
  nh.param("maxSurfaceAngle", maxSurfaceAngle, 45.0);
  nh.param("minAngle", minAngle, 10.0);
  nh.param("maxAngle", maxAngle, 120.0);
  nh.param("normalConsistency", normalConsistency, false);

  // Convert angles to radians
  maxSurfaceAngle = degToRad(maxSurfaceAngle);
  minAngle = degToRad(minAngle);
  maxAngle = degToRad(maxAngle);

  CloudProcessingNodeMGFT c(topicIn/*, topicOut*/);
  ROS_INFO_STREAM("Writing to files: " << (toggleWriteToFile ? "enabled" : "disabled") << "\n"
                  << "Search radius: " << searchRadius << "\n"
                  << "\u039c: " << mu << "\n"
                  << "Maximum nearest neighbours: " << maxNN << "\n"
                  << "Maximum surface angle: " << maxSurfaceAngle << "\n"
                  << "Minimum angle: " << minAngle << "\n"
                  << "Maximum angle: " << maxAngle
                  << "Normal consistency: " << (normalConsistency ? "enabled" : "disabled") << "\n");
  c.setWritingToFile(toggleWriteToFile);
  c.setSearchRadius(searchRadius);
  c.setMu(mu);
  c.setMaxNN(maxNN);
  c.setMaxSurfaceAngle(maxSurfaceAngle);
  c.setMinAngle(minAngle);
  c.setMaxAngle(maxAngle);
  c.setNormalConsistency(normalConsistency);

  while(nh.ok())
    ros::spin();

  return 0;
}

