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
// ...
// ROS - Publishing
#include <pcl_ros/publisher.h>

// PCL
// PCL - Misc
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// ...
// Misc
#include <sstream>
#include <string>
// ...

/*
 * Naming conventions for pmd_camboard_nano project: 
 * 
 *  # Class name: "class CloudProcessingNodeABREVIATION"
 *      Example: "class CloudProcessingNodeNBSF" with NBSF = Nurbs B-Spline Fitting (file: pmd_camboard_nano_cloud_nurbs_bspline_fitting.cpp)
 *  # Output PCD file: "cloud_ABREVIATION_" or "cloud_" + some name that indicates the contents of the PCD file
 *      Example: "cloud_nbsf_"
 */
class CloudProcessingNodeTEMPLATE
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
  // ...
  
public:
  CloudProcessingNodeNBSF(std::string topicIn, std::string topicOut)
    : fileIdx(0),
      //fileSmoothSurfOutputIdx(0),
      //toggleWritingToFile(false)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeTEMPLATE::subCallback, this);
    pub.advertise(nh, topicOut, 1);
  }

  ~CloudProcessingNodeTEMPLATE()
  {
    sub.shutdown();
  }

  void setWritingToFile(bool _toggle) {
    toggleWritingToFile = _toggle;
  }

  // ...
  
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

    // Do something with the PCL cloud
    // ...
    
    
    // Optional: write result to a binary compressed PCD file
    if(toggleWritingToFile)
    {
      //std::string path = "/home/USER/catkin_ws/devel/lib/pmd_camboard_nano/";
      std::string path = "~/catkin_ws/devel/lib/pmd_camboard_nano/";
      ss << path << "cloud_template_" << fileIdx << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(ss.str(), *p);
      fileIdx++;
      ss.str("");
    }

    
    // Convert result back to ROS message and publish it
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*p, output);
    pub.publish(output);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_template");
  ros::NodeHandle nh("~");
  std::string topicIn = "/camera/points"; //The "raw" output from the PMD camera (raw means unfiltered)
  std::string topicOut = "points_template";
  // ...

  nh.param("write_to_file", toggleWriteToFile, false);

  CloudProcessingNodeTEMPLATE c(topicIn, topicOut);
  ROS_INFO_STREAM("Writing to files " << toggleWriteToFile ? "activated" : "deactivated");
  c.setWritingToFile(toggleWriteToFile);

  while(nh.ok())
    ros::spin();

  return 0;
}

