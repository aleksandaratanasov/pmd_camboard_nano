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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL - Mesh generation
#ifdef BUILD_WITH_NURBS
  // Use NURBS
  #include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
  #include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
  #include <pcl/surface/on_nurbs/triangulation.h>
#else
  // Use fast triangulation
  #include <pcl/kdtree/kdtree_flann.h>
  #include <pcl/features/normal_3d.h>
  #include <pcl/surface/gp3.h>
#endif

// Misc
#include <sstream>
#include <string>

class CloudProcessingnodeMG
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;

private:
  sensor_msgs::PointCloud2 cloud;

  // For writing to files
  u_int64_t fileIdx;
  std::ostringstream ss;
  bool toggleWritingToFile;

public:
  CloudProcessingnodeMG(std::string topicIn, std::string topicOut)
    : fileIdx(0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingnodeMG::subCallback, this);
    pub.advertise(nh, topicOut, 1);
  }

  ~CloudProcessingnodeMG()
  {
    sub.shutdown();
  }

  void setWritingToFile(bool _toggle) {
    toggleWritingToFile = _toggle;
  }

  void setPolynomialFit(bool _polynomialFit)
  {
    //...
  }

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
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*msg, pclCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr p(new pcl::PointCloud<pcl::PointXYZ>(pclCloud));

    // ...

    if(toggleWritingToFile)
    {
      std::string path = "/home/latadmin/catkin_ws/devel/lib/pmd_camboard_nano/";
      ss << path << "cloud_mesh_generator_" << fileIdx << ".3dm"; // TODO Use some user-friendlier format for the mesh (3dm doesn't seem to be not that popular)
      pcl::io::savePCDFileBinaryCompressed(ss.str(), *p);
      ROS_INFO_STREAM("Writing to file \"" << ss.str() << "\"");
      fileIdx++;
      ss.str("");
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*p, output);
    pub.publish(output);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_mesh_generator");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_ssne";
  std::string topicOut = "points_mg";
  bool toggleWriteToFile;

  nh.param("write_to_file", toggleWriteToFile, false);

  CloudProcessingnodeMG c(topicIn, topicOut);
  ROS_INFO_STREAM("Writing to files " << toggleWriteToFile ? "activated" : "deactivated");
  c.setWritingToFile(toggleWriteToFile);
  //ROS_INFO_STREAM((polynomialFit ? "Enabling" : "Disabling") << " polynomial fit and setting search radius to " << searchRadius);
  // ...

  while(nh.ok())
    ros::spin();

  return 0;
}

