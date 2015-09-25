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
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

// Misc
#include <sstream>
#include <string>

class CloudProcessingNodeSSNE
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
  bool polynomialFit; // polynomial fit value is true if the surface and normal are approximated using a polynomial
  double searchRadius; // sphere radius that is to be used for determining the k-nearest neighbors used for fitting

public:
  CloudProcessingNodeSSNE(std::string topicIn, std::string topicOut)
    : fileIdx(0)
      //fileSmoothSurfOutputIdx(0),
      //toggleWritingToFile(false),
      //polynomialFit(false),
      //searchRadius(0.03)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeSSNE::subCallback, this);
    pub.advertise(nh, topicOut, 1);
  }

  ~CloudProcessingNodeSSNE()
  {
    sub.shutdown();
    pub.shutdown();
  }

  void setWritingToFile(bool _toggle) {
    toggleWritingToFile = _toggle;
  }

  void setPolynomialFit(bool _polynomialFit)
  {
    polynomialFit = _polynomialFit;
  }

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

    // Source: http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_%28basic%29#Removing_NaNs
    // Removing NaN points from cloud (if those are not remove the KD-Tree and MLS will fail)
    std::vector<int> mapping; //
    pcl::removeNaNFromPointCloud(*p, *p, mapping);

    // Optional: write filtered cloud to a binary compressed PCD
    if(toggleWritingToFile)
    {
      std::string path = "";
      ss << path << "cloud_reduced_outliers_" << fileIdx << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(ss.str(), *p);
      fileIdx++;
      ss.str("");
    }

    // TODO See if this node can be split into two - one for the normal estimation and another for the surface smoothing
    // The mesh generator node requires similar way of computing the normals so maybe there is a chance to make this process more flexible
    // Source: http://pointclouds.org/documentation/tutorials/resampling.php
    // Estimate normals and apply smoothing at the end before the mesh generation
    // Creating the KD-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    // Set parameters
    mls.setInputCloud (p);//(FINAL_CLOUD_FULL_REGISTERED_FRAMES);
    mls.setPolynomialFit (false);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    // Reconstruct
    mls.process (mls_points);

    if(toggleWritingToFile)
    {
      std::string path = "";
      ss << path << "cloud_smooth_surface_" << fileIdx << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(ss.str(), mls_points);
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
  ros::init (argc, argv, "cloud_surface_smoothing_normal_estimation");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_sor";
  std::string topicOut = "points_ssne";
  bool toggleWriteToFile;
  bool polynomialFit;
  double searchRadius;

  //nh.param("subscribeTo", topicIn);
  //nh.param("publish", topicOut);
  nh.param("write_to_file", toggleWriteToFile, false);
  nh.param("polynomialFit", polynomialFit, false);
  nh.param("searchRadius", searchRadius, 0.03);

  CloudProcessingNodeSSNE c(topicIn, topicOut);
  ROS_INFO_STREAM("Writing to files " << (toggleWriteToFile ? "activated" : "deactivated"));
  c.setWritingToFile(toggleWriteToFile);
  ROS_INFO_STREAM((polynomialFit ? "Enabling" : "Disabling") << " polynomial fit and setting search radius to " << searchRadius);
  c.setPolynomialFit(polynomialFit);
  c.setSearchRadius(searchRadius);

  while(nh.ok())
    ros::spin();

  return 0;
}

