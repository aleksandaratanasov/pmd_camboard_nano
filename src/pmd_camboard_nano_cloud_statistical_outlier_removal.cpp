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

class CloudSubscriberSOR
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
  // For the statistical outlier removal
  int meanK; // number of nearest neighbors to use for mean distance estimation
  double stdDevMulThresh; // standard deviation multiplier for the distance threshold calculation

public:
  CloudSubscriberSOR(std::string topicIn, std::string topicOut)
    : fileIdx(0),
      //fileSmoothSurfOutputIdx(0),
      toggleWritingToFile(false),
      meanK(50),
      stdDevMulThresh(1.0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudSubscriberSOR::subCallback, this);
    pub.advertise(nh, topicOut, 1);
  }

  ~CloudSubscriberSOR()
  {
    sub.shutdown();
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
    ROS_INFO_STREAM("Filtering completed. a cloud message with " << (msg->height*msg->width - pclCloud_filtered->height*pclCloud_filtered->width) << " points as outliers leaving " << pclCloud_filtered->height * pclCloud_filtered->width << " in total");

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
      std::string path = "/home/latadmin/catkin_ws/devel/lib/pmd_camboard_nano/";
      ss << path << "cloud_reduced_outliers_" << fileIdx << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(ss.str(), *pclCloud_filtered);
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
  ros::NodeHandle nh;
  std::string topicIn = "/camera/points";
  std::string topicOut = "/camera/cloud_statistical_outlier_removal";
  bool toggleWriteToFile = false;
  int meanK = 50;
  double stdDevMulThresh = 1.0;

  nh.param("topicIn", topicIn);
  nh.param("topicOut", topicOut);
  nh.param("write_to_file", toggleWriteToFile);
  nh.param("meanK", meanK);
  nh.param("stdDevMulThresh", stdDevMulThresh);

  CloudSubscriberSOR c(topicIn, topicOut);
  ROS_INFO_STREAM("Writing to files " << toggleWriteToFile ? "activated" : "deactivated");
  c.setWritingToFile(toggleWriteToFile);
  ROS_INFO_STREAM("Setting mean K to " << meanK << " and standard deviation multipler threshold to " << stdDevMulThresh);
  c.setMeanK(meanK);
  c.setStdDevMulThres(stdDevMulThresh);

  while(nh.ok())
    ros::spin();

  return 0;
}

