#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
// For the pairwise registration
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

// For writing a PCD file (in order to view it later)
#include <pcl/io/pcd_io.h>

// Boost
#include <boost/make_shared.hpp>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

ros::Publisher pub;

void processData(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO_STREAM(ros::this_node::getName() << " : Received cloud with " << msg->height * msg->width << " points");
  ROS_INFO_STREAM(ros::this_node::getName() << " : Converting sensor_msgs::PointCloud2 to PCLPointCloud2");
  pcl::PCLPointCloud2 pclCloud;
  pcl_conversions::toPCL(*msg, pclCloud);
  ROS_INFO_STREAM(ros::this_node::getName() << " : PCL point cloud contains " << pclCloud.height * pclCloud.width << " points");

  ROS_INFO("Relaying point cloud");
  pub.publish(msg);
}

int main (int argc, char** argv)
{
  ROS_INFO("Relaying point cloud data");
  // Initialize ROS
  ros::init (argc, argv, "pmd_camboard_nano_cloud_relay");
  ros::NodeHandle nh;
  //pmd_camboard_nano::PMDCamboardNanoCloudHandler ch;
  //ch.spin();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/points_unrectified", 1, processData);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/cloud_relayed", 10);

  ros::Rate r(ros::Duration(5, 0)); //5s tact
  while(nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }
  // Spin
  //ros::spin();

  return 0;
}
