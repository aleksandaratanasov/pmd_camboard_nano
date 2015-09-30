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
#include <ros/console.h>
// ...
// ROS - Publishing
#include <pcl_ros/publisher.h>

// PCL
// PCL - Misc
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL - Iterative Closest Point
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// Misc
#include <sstream>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

class CloudProcessingNodeICP
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;

private:
  // Define a new point representation for < x, y, z, curvature >
  class CustomPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
  {
  public:
    CustomPointRepresentation ()
    {
      // Define the number of dimensions
      pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
    {
      // < x, y, z, curvature >
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.curvature;
    }
  };

  sensor_msgs::PointCloud2 cloud;

  // For writing to files
  u_int64_t fileIdx;
  std::ostringstream ss;
  bool toggleWritingToFile;
  unsigned int fillStatus;
  unsigned int capacity;
  double maxCorrespondenceDistance;
  int maxIterations;
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> *clouds;

  void pairAlign(pcl::PointCloud<pcl::PointNormal>::Ptr src, const pcl::PointCloud<pcl::PointNormal>::Ptr dst, pcl::PointCloud<pcl::PointNormal>::Ptr temp, Eigen::Matrix4f &final_transform) {
    // Instantiate our custom point representation (defined above) ...
    CustomPointRepresentation point_representation;
    // Weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1., 1., 1., 1.};
    point_representation.setRescaleValues(alpha);

    // Align
    // TODO move IterativeClosestPointNonLinear object to a class member so that it won't be created all the time (settings are always the same so no need for that)
    ROS_INFO("Setting up non linear ICP");
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences in source and destination to 10cm
    // TODO Move ICP's max correspondence distance to the launch file since it is based on the dataset
    reg.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    // Set the point presentation
    reg.setPointRepresentation(boost::make_shared<const CustomPointRepresentation>(point_representation));
    reg.setInputSource(src);
    reg.setInputTarget(dst);

    // Loop and optimize
    ROS_INFO("Running optimization for a single pair");
    Eigen::Matrix4f ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = src;
    reg.setMaximumIterations(maxIterations); // TODO Move max iterations to launch file

    for(int i = 0; i < 30; ++i) { // See what this 30 is. Isn't his maxIterations?
      src = reg_result;
      // Estimate
      reg.setInputSource(src);
      reg.align(*reg_result);

      // Accumulate transformation between each iteration
      ti = reg.getFinalTransformation() * ti;

      // Check if difference current and previous transformation is smaller than the threshold
      // Refine the process by reducing the maximal correspondence distance
      if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
        reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - .001);

      prev = reg.getLastIncrementalTransformation ();
    }

    // Get transformation from target to source
    targetToSource = ti.inverse();
    // Transform target back to source frame
    pcl::transformPointCloudWithNormals(*dst, *temp, targetToSource);
    // Add source to transformed destination
    *temp += *src;
    final_transform = targetToSource;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr runICP() {
    ROS_INFO("Initiating ICP");
    // Run ICP
    pcl::PointCloud<pcl::PointNormal>::Ptr result(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr src, dst;
    Eigen::Matrix4f globalTransform = Eigen::Matrix4f::Identity(), pairTransform;

    // Downsample (here we have < 20000 points per cloud so no need for that)
    ROS_INFO("Running ICP for %lu clouds", clouds->size());
    for(unsigned int cloud = 1; cloud < clouds->size(); cloud++) {
      src = clouds->at(cloud - 1);
      dst = clouds->at(cloud);
      pcl::PointCloud<pcl::PointNormal>::Ptr temp(new pcl::PointCloud<pcl::PointNormal>);
      pairAlign(src, dst, temp, pairTransform);

      // Transform current pair into global transform
      pcl::transformPointCloudWithNormals(*temp, *result, globalTransform);

      // Update global transform based on the transform of the current pair
      globalTransform = globalTransform * pairTransform;
      ROS_INFO("Done %u out of %lu", cloud, clouds->size());
    }

    return result;
  }

public:
  CloudProcessingNodeICP(std::string topicIn, std::string topicOut, unsigned int _capacity)
    : fileIdx(0), fillStatus(0), capacity(_capacity)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeICP::subCallback, this);
    pub.advertise(nh, topicOut, 1);
    clouds = new std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>(capacity);
  }

  ~CloudProcessingNodeICP()
  {
    sub.shutdown();
    delete clouds;
  }

  void setWritingToFile(bool _toggle) { toggleWritingToFile = _toggle; }

  void setCapacity(unsigned int _capacity) {
    capacity = _capacity;
    clouds->reserve(capacity);
  }

  void setMaxCorrespondenceDist(double _maxCorrespondenceDistance) { maxCorrespondenceDistance = _maxCorrespondenceDistance; }

  void setMaxIterations(int _maxIterations) { maxIterations = _maxIterations; }
  
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
    pcl::PointCloud<pcl::PointNormal>::Ptr p(new pcl::PointCloud<pcl::PointNormal>(pclCloud));

    // TODO Add multithreading here - fill a container, pass it onto a thread (copy of it), repeat
    // Fill the vector with point clouds until the limit is reached
    if(fillStatus < capacity) {
      ROS_INFO_STREAM("Current capacity: " << fillStatus << " out of " << capacity);
      clouds->at(fillStatus) = p;  // Add received cloud to the sequence
      fillStatus++;
    }
    else {
      // Run the vector of clouds through ICP and generate the final cloud
      // This is triggered once enough samples have been gathered
      // Source: http://pointclouds->org/documentation/tutorials/pairwise_incremental_registration.php
      // Source: http://pointclouds->org/documentation/tutorials/interactive_icp.php
      pcl::PointCloud<pcl::PointNormal>::Ptr p_icp(runICP());
      ROS_INFO("Cleaning sample sequence");
      fillStatus = 0;

      // Optional: write result to a binary compressed PCD file
      if(toggleWritingToFile)
      {
        std::string path = "/home/redbaron/catkin_ws/src/pmd_camboard_nano/samples/temp/";
        ss << path << "cloud_icp_" << fileIdx << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(ss.str(), *p_icp);
        fileIdx++;
        ss.str("");
      }

      // Convert result back to ROS message and publish it
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*p_icp, output);
      pub.publish(output);
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_icp");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_ne";  // Currently the implementation of the CloudProcessingNodeICP uses only clouds of type pcl::PointNormal so topicIn has to be of the same type
  std::string topicOut = "points_icp";
  bool toggleWriteToFile;
  int capacity;
  double maxCorrespondenceDistance;
  int maxIterations;

  if(capacity < 2) {
    ROS_WARN("Capacity < 2. Falling back to default: 30");
    capacity = 30;
  }

  nh.param("write_to_file", toggleWriteToFile, false);
  nh.param("capacity", capacity, 30); // capture a maximum of 30 frames before triggering the ICP
  nh.param("maxCorrespondenceDist", maxCorrespondenceDistance, .1);
  nh.param("maxIterations", maxIterations, 2);

  CloudProcessingNodeICP c(topicIn, topicOut, capacity);
  ROS_INFO_STREAM("Writing to files: " << (toggleWriteToFile ? "enabled" : "disabled") << "\n"
                  << "Capacity: " << capacity << "\n"
                  << "Max. correspondence distance: " << maxCorrespondenceDistance << "\n"
                  << "Max. iterations: " << maxIterations);
  c.setWritingToFile(toggleWriteToFile);
  //c.setCapacity((unsigned int)capacity); // Can be used to adjust the capacity later on
  c.setMaxCorrespondenceDist(maxCorrespondenceDistance);
  c.setMaxIterations(maxIterations);

  while(nh.ok())
    ros::spin();

  return 0;
}
