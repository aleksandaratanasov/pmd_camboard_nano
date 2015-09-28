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
// PCL - For the removal of outliers
#include <pcl/filters/statistical_outlier_removal.h>

// Misc
#include <sstream>
#include <string>

class CloudProcessingNodeSOR
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
  int meanK; // number of nearest neighbors to use for mean distance estimation
  double stdDevMulThresh; // standard deviation multiplier for the distance threshold calculation

public:
  CloudProcessingNodeSOR(std::string topicIn, std::string topicOut)
    : fileIdx(0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeSOR::subCallback, this);
    pub.advertise(nh, topicOut, 1);
  }

  ~CloudProcessingNodeSOR()
  {
    sub.shutdown();
    pub.shutdown();
  }

  void setWritingToFile(bool _toggle) {
    toggleWritingToFile = _toggle;
  }

  void setMeanK(int _meanK)
  {
    meanK = _meanK;
  }

  void setStdDevMulThres(double _stdDevMulThresh)
  {
    stdDevMulThresh = _stdDevMulThresh;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Source: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
    // Use a statistical outlier removal to reduce the number of outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // Configure the removal procedure
    sor.setInputCloud(p);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdDevMulThresh);
    ROS_INFO("Filtering cloud data");
    // Remove the outliers
    sor.filter(*pclCloud_filtered);
    ROS_INFO_STREAM("Filtering completed. Removed " << (msg->height*msg->width - pclCloud_filtered->height*pclCloud_filtered->width) << " points as outliers and left " << pclCloud_filtered->height * pclCloud_filtered->width << " inliers");

    // Negating an applying filtering returns the outliers if required
    //sor.setNegative(true);
    //sor.filter(*pclCloud_filtered);

    // Source: http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_%28basic%29#Removing_NaNs
    // Removing NaN points from cloud (if those are not remove the KD-Tree and MLS will fail
    std::vector<int> mapping; //
    pcl::removeNaNFromPointCloud(*pclCloud_filtered, *pclCloud_filtered, mapping);

    // Optional: write filtered cloud to a binary compressed PCD
    if(toggleWritingToFile)
    {
      std::string path = "";
      ss << path << "cloud_reduced_outliers_" << fileIdx << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(ss.str(), *pclCloud_filtered);
      ROS_INFO_STREAM("Writing to file \"" << ss.str() << "\"");
      fileIdx++;
      ss.str("");
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pclCloud_filtered, output);
    pub.publish(output);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_statistical_outlier_removal");
  ros::NodeHandle nh("~");
  std::string topicIn = "/camera/points";
  std::string topicOut = "points_sor";
  bool toggleWriteToFile = false;
  int meanK;
  double stdDevMulThresh;

  //nh.param("topicIn", topicIn);
  //nh.param("topicOut", topicOut);
  nh.param("write_to_file", toggleWriteToFile, false);
  nh.param("meanK", meanK, 50);
  nh.param("stdDevMulThresh", stdDevMulThresh, 1.0);

  ROS_INFO("Subscribed to \"%s\"", topicIn.c_str());
  ROS_INFO("Publishing to \"%s\"", topicOut.c_str());

  CloudProcessingNodeSOR c(topicIn, topicOut);
  ROS_INFO_STREAM("Writing to files: " << (toggleWriteToFile ? "enabled" : "disabled") << "\n"
                  << "Mean K: " << meanK << "\n"
                  << "Standard deviation threshold: " << stdDevMulThresh
                  );
  c.setWritingToFile(toggleWriteToFile);
  c.setMeanK(meanK);
  c.setStdDevMulThres(stdDevMulThresh);

  while(nh.ok())
    ros::spin();

  return 0;
}

