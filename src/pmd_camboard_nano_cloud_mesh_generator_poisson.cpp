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
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL - Mesh generation
// Use Poisson surface reconstruction
#include <pcl/surface/poisson.h>

// Misc
#include <sstream>
#include <string>

class CloudProcessingNodeMGPSR
{
  typedef pcl::PointXYZ Point;
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;

private:
  sensor_msgs::PointCloud2 cloud;

  // For writing to files
  u_int64_t fileIdx;
  std::ostringstream ss;
  bool toggleWritingToFile;
  int depth;

public:
  CloudProcessingNodeMGPSR(std::string topicIn)
    : fileIdx(0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeMGPSR::subCallback, this);
  }

  ~CloudProcessingNodeMGPSR()
  {
    sub.shutdown();
//    pub.shutdown();
  }

  void setWritingToFile(bool toggle) { toggleWritingToFile = toggle; }

  void setDepth(int _depth) { depth = _depth; }

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

    pcl::Poisson<pcl::PointNormal> poisson;
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.setDepth(depth);
    poisson.setInputCloud(p_with_normals);
    poisson.reconstruct(*mesh);
    ROS_INFO_STREAM("Number of polygons: " << mesh->polygons.size());

    if(toggleWritingToFile)
    {
      std::string path = "/home/redbaron/catkin_ws/src/pmd_camboard_nano/samples/temp/";
      ss << path << "cloud_mesh_generator_poisson_" << fileIdx << ".stl";
      pcl::io::savePolygonFileSTL(ss.str(), *mesh);
      ROS_INFO_STREAM("Writing to file \"" << ss.str() << "\"");
      fileIdx++;
      ss.str("");
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_mesh_generator_poisson");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_ne";
  bool toggleWriteToFile;
  int depth;

  nh.param("write_to_file", toggleWriteToFile, false);
  nh.param("depth", depth, 10);

  CloudProcessingNodeMGPSR c(topicIn);
  ROS_INFO_STREAM("Writing to files: " << (toggleWriteToFile ? "enabled" : "disabled") << "\n"
                  << "Depth: " << depth);
  c.setWritingToFile(toggleWriteToFile);
  c.setDepth(depth);

  while(nh.ok())
    ros::spin();

  return 0;
}

