/*#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
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
// Filtering
#include <pcl/filters/statistical_outlier_removal.h>

// For writing a PCD file (in order to view it later)
#include <pcl/io/pcd_io.h>

// Boost
#include <boost/make_shared.hpp>

// Misc
#include <sstream>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
*/
/*
namespace pmd_camboard_nano
{

  class PMDCamboardNanoCloudHandler
  {
    public:
      PMDCamboardNanoCloudHandler()
      {
        ROS_INFO("Subscribing to cloud topic");
        sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/points", 1, boost::bind(&PMDCamboardNanoCloudHandler::processData, this, _1));
      }

      ~PMDCamboardNanoCloudHandler()
      {
        sub.shutdown();
      }

      void spin()
      {
        while(nh.ok());
      }

    private:
      ros::NodeHandle nh;
      ros::Subscriber sub;
      sensor_msgs::PointCloud2ConstPtr cloudMsg;
      PointCloudPtr cloud;

      void processData(const sensor_msgs::PointCloud2ConstPtr& input)
      {
        ROS_INFO("Hello!");
        ROS_INFO_STREAM(ros::this_node::getName() << " | " << input->header.stamp << " : Received cloud with " << input->height * input->width << " points");
      }
  };
}
*/

//unsigned long int fileIdx = 0;

// TODO publish two point clouds from the received cloud - one that shows only the outliers and the other - the filtered points
//ros::Publisher pubOutliers;
//ros::Publisher pubFiltered;

//void processData(const sensor_msgs::PointCloud2ConstPtr& msg)
//{
  ////////////////////////////////////// OLDEST ///////////////////////////////////////
  /*ROS_INFO_STREAM(ros::this_node::getName() << " : Received cloud with " << msg->height * msg->width << " points");
  ROS_INFO_STREAM(ros::this_node::getName() << " : Converting sensor_msgs::PointCloud2 to PCLPointCloud2");
  pcl::PCLPointCloud2 temp;
  pcl_conversions::toPCL(*msg, temp);
  PointCloudPtr pclCloud(new pcl::PointCloud<pcl::PointXYZ>(temp.width, temp.height));
  pcl::fromPCLPointCloud2(temp, *pclCloud);
  ROS_INFO_STREAM(ros::this_node::getName() << " : PCL point cloud contains " << pclCloud->height * pclCloud->width << " points");
  //ROS_INFO_STREAM(ros::this_node::getName() << " : PCL point cloud contains " << pclCloud.height * pclCloud.width << " points");

  // Process the cloud here
  std::ostringstream ss;
  ss << "cloud_" << fileIdx << ".pcd";
  std::string file(ss.str());
  pcl::io::savePCDFileBinaryCompressed(file, *pclCloud);
  fileIdx++;
  ss.str("");
  */

  //////////////////////////////////////// OLD ////////////////////////////////////////
  // Convert message to data structure compatible with the PCL library
  //
  //    sensor_msgs::PointCloud2 --> pcl::PCLPointCloud2 --> pcl::PointCloud<pcl::PointXYZ>
  //
  // FIXME Conversion below doesn't work properly. Whenever I try to use the pclCloud after it I get a segmentation fault
  /*ROS_INFO_STREAM(ros::this_node::getName() << " : Received cloud with " << msg->height * msg->width << " points");
  ROS_INFO_STREAM(ros::this_node::getName() << " : Converting sensor_msgs::PointCloud2 to PCLPointCloud2");
  pcl::PCLPointCloud2 msgToPc2;
  pcl_conversions::toPCL(*msg,msgToPc2);
  ROS_INFO_STREAM(ros::this_node::getName() << " : Converting PCLPointCloud2 to PointCloud<pcl::PointXYZ>");
  PointCloudPtr pclCloud(new PointCloud); //(new PointCloud(msg->width, msg->height));
  pcl::fromPCLPointCloud2(msgToPc2,*pclCloud);
  ROS_INFO_STREAM(ros::this_node::getName() << " : Conversion completed. Result contains " << pclCloud->height * pclCloud->width << " points");*/

  // Write to file
  // FIXME segmentation faults when using pclCloud!!!
  /*std::ostringstream ss;
  ss << "/home/latadmin/catkin_ws/devel/lib/pmd_camboard_nano/cloud_" << fileIdx << ".pcd";
  pcl::io::savePCDFileBinaryCompressed(ss.str(), *pclCloud);
  fileIdx++;
  ss.str("");*/

  // Filter outliers
  /*PointCloudPtr pclCloud_filtered(new PointCloud); //(new PointCloud(pclCloud->width, pclCloud->height));
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(pclCloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*pclCloud_filtered);

  // Convert back to sensor_msgs::PointCloud2 and publish filtered cloud
  sensor_msgs::PointCloud2 filteredMsg;
  pcl::toROSMsg(*pclCloud_filtered, filteredMsg);
  pubFiltered.publish(filteredMsg);*/

  //////////////////////////////////////// NEW ////////////////////////////////////////
  /*pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*msg, pclCloud);
  //PointCloudPtr pclCloud_filtered(new PointCloud); //(new PointCloud(pclCloud->width, pclCloud->height));
  //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  ROS_INFO("FILTERING");
  ROS_INFO_STREAM("POINTS: " << pclCloud.height * pclCloud.width);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr p(new pcl::PointCloud<pcl::PointXYZ>(pclCloud)); //<------------- double free or corruption HERE!!!
  //sor.setInputCloud(p);
  //sor.setMeanK(50);
  //sor.setStddevMulThresh(1.0);
  //sor.filter(*pclCloud_filtered);*/
//}
/*
main (int argc, char** argv)
{
  ROS_INFO("Starting cloud handler to process PMD Camboard Nano point cloud data");
  // Initialize ROS
  ros::init (argc, argv, "pmd_camboard_nano_cloud_handler");
  ros::NodeHandle nh;
  //pmd_camboard_nano::PMDCamboardNanoCloudHandler ch;
  //ch.spin();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/points_unrectified", 1, processData);//<sensor_msgs::PointCloud2Ptr>("/camera/points_unrectified", 1, processData);
  //pubFiltered = nh.advertise<sensor_msgs::PointCloud2>("/camera/cloud_filtered", 10);

  ros::Rate r(ros::Duration(5, 0)); //5s tact
  while(nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }
  // Spin
  //ros::spin();

  return 0;
}
*/

































#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/publisher.h>

// Misc
#include <sstream>
#include <string>

class CloudSubscriber
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  u_int64_t fileIdx;

private:
  sensor_msgs::PointCloud2 cloud;

public:
  CloudSubscriber()
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/points", 5, &CloudSubscriber::subCallback, this);
    fileIdx = 0;
  }

  ~CloudSubscriber()
  {
    sub.shutdown();
  }

  void subCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if(msg->data.empty())
    {
      ROS_WARN("Received an empty cloud message. Skipping further processing");
      return;
    }

    ROS_INFO_STREAM("Received a cloud message with " << msg->height * msg->width << " points");
    ROS_INFO("Converting ROS cloud message to PCL compatible data structure");
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*msg, pclCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr p(new pcl::PointCloud<pcl::PointXYZ>(pclCloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(p);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    ROS_INFO("Filtering cloud data");
    // Remove the outliers
    sor.filter(*pclCloud_filtered);
    ROS_INFO_STREAM("Filtering completed. a cloud message with " << (msg->height*msg->width - pclCloud_filtered->height*pclCloud_filtered->width) << " points as outliers leaving " << pclCloud_filtered->height * pclCloud_filtered->width << " in total");

    // Negating an applying filtering returns the outliers
    //sor.setNegative(true);
    //sor.filter(*pclCloud_filtered);

    std::ostringstream ss;
    std::string path = "/home/latadmin/catkin_ws/devel/lib/pmd_camboard_nano/pcl_samples/";
    ss << path << "cloud_" << fileIdx << ".pcd";
    pcl::io::savePCDFileBinaryCompressed(ss.str(), *pclCloud_filtered);
    fileIdx++;
    ss.str("");
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_subscriber");
  ros::NodeHandle nh;

  CloudSubscriber c;

  while(nh.ok())
    ros::spin();

  return 0;
}

