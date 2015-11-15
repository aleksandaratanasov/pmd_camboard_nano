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
// PCL - For the normals estimation and surface smoothing
#include "config.h"
#ifdef ENABLE_OPENMP
  #include <pcl/features/normal_3d_omp.h>
#else
  #include <pcl/features/normal_3d.h>
#endif

// Misc
#include <sstream>
#include <string>

class CloudProcessingNodeNE
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
  double searchRadius; // sphere radius that is to be used for determining the k-nearest neighbours used for fitting

public:
  CloudProcessingNodeNE(std::string topicIn, std::string topicOut)
    : fileIdx(0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeNE::subCallback, this);
    pub.advertise(nh, topicOut, 1);
  }

  ~CloudProcessingNodeNE()
  {
    sub.shutdown();
    pub.shutdown();
  }

  void setWritingToFile(bool toggle) { toggleWritingToFile = toggle; }

  void setSearchRadius(double _searchRadius)
  {
    searchRadius = _searchRadius;
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

    // Source: http://pointclouds.org/documentation/tutorials/normal_estimation.php
    // Create the normal estimation class, and pass the input dataset to it
#ifdef ENABLE_OPENMP
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(2);
#else
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
#endif
    ne.setInputCloud(p);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(searchRadius);
    Eigen::Vector4f centroid;
    compute3DCentroid(*p, centroid);  // Compute the 3D centroid (X,Y and Z) and produce a vector which will be used for setting the view point
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]); // The view point is essential for the estimation of the normals

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Compute the features
    ne.compute(*normals);

    if(normals->points.size() != p->points.size()) {
      ROS_ERROR_STREAM("Number of points in estimated cloud with normals is unequalt to the original cloud:" << "\n"
                       << "Original: " << p->points.size() << "\n"
                       << "Normals: " << normals->points.size());
      return;
    }

    // Merge normals with the cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*p, *cloud_with_normals);
    pcl::copyPointCloud(*normals, *cloud_with_normals);

    // Optional: write cloud with normals to a binary compressed PCD
    if(toggleWritingToFile)
    {
      std::string path = "/home/redbaron/catkin_ws/src/pmd_camboard_nano/samples/temp/";
      ss << path << "cloud_normals_" << fileIdx << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(ss.str(), *cloud_with_normals);
      //pcl::io::savePCDFileASCII(ss.str(), *cloud_with_normals);
      ROS_INFO_STREAM("Writing to file \"" << ss.str() << "\"");
      fileIdx++;
      ss.str("");
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_with_normals, output);
    pub.publish(output);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_normal_estimation");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_sor";
  std::string topicOut = "points_ne";
  bool toggleWriteToFile = false;
  double searchRadius = 0.;

  nh.param("write_to_file", toggleWriteToFile, false);
  nh.param("searchRadius", searchRadius, .03);

  CloudProcessingNodeNE c(topicIn, topicOut);
  ROS_INFO_STREAM("Writing to files: " << (toggleWriteToFile ? "enabled" : "disabled") << "\n"
                  << "Search radius: " << searchRadius);
  c.setWritingToFile(toggleWriteToFile);
  c.setSearchRadius(searchRadius);

  while(nh.ok())
    ros::spin();

  return 0;
}

