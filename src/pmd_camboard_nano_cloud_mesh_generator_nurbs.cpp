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
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL - Mesh generation
// Use NURBS
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

// Misc
#include <sstream>
#include <string>

// TODO See how to make NURBS mesh generation support pcl::PointNormal clouds so that this node can be attached to the results of the normale estimation node

class CloudProcessingNodeMGNURBS
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

  void PointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
  {
    for (unsigned i = 0; i < cloud->size(); i++)
    {
      Point &p = cloud->at(i);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z))
        data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
  }

public:
  CloudProcessingNodeMGNURBS(std::string topicIn)
    : fileIdx(0)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>(topicIn, 5, &CloudProcessingNodeMGNURBS::subCallback, this);
  }

  ~CloudProcessingNodeMGNURBS()
  {
    sub.shutdown();
  }

  void setWritingToFile(bool toggle) { toggleWritingToFile = toggle; }

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
    // NURBS
    // Source: http://pointclouds.org/documentation/tutorials/bspline_fitting.php
    // The B-spline surface
    pcl::on_nurbs::NurbsDataSurface data;
    PointCloud2Vector3d (p, data.interior);

    // Set the parameters for the fitting
    unsigned order (3);
    unsigned refinement (5);
    unsigned iterations (10);
    unsigned mesh_resolution (256);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    // Initialize
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);
    //  fit.setQuiet(false); // enable/disable debug output

    // Setup the mesh for the output
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, *mesh, mesh_resolution);

    // Surface refinement
    ROS_INFO("Surface refinement");
    for(unsigned i = 0; i < refinement; i++)
    {
      ROS_INFO("Refinement %d", i);
      fit.refine(0);
      fit.refine(1);
      fit.assemble(params);
      fit.solve();
      pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    }

    // surface fitting with final refinement level
    ROS_INFO("Surface fitting with final refinement level");
    for (unsigned i = 0; i < iterations; i++)
    {
      ROS_INFO("Iteration %d", i);
      fit.assemble(params);
      fit.solve();
      pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    }

    // The B-spline curve
    // Set the parameters for the fitting
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 200;
    curve_params.accuracy = 1e-3;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    // Initialisation (circular)
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back (true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);

    // Curve fitting
    ROS_INFO("Fit curve");
    pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
    // curve_fit.setQuiet(false); // enable/disable debug output
    curve_fit.fitting(curve_params);

    // Triangulation of trimmed surface
    ROS_INFO("Triangulating trimmed surface");
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, *mesh, mesh_resolution);

    // Save trimmed B-spline surface
    if(fit.m_nurbs.IsValid())
    {
      ONX_Model model;
      ONX_Model_Object& surf = model.m_object_table.AppendNew();
      surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
      surf.m_bDeleteObject = true;
      surf.m_attributes.m_layer_index = 1;
      surf.m_attributes.m_name = "surface";

      ONX_Model_Object& curv = model.m_object_table.AppendNew();
      curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
      curv.m_bDeleteObject = true;
      curv.m_attributes.m_layer_index = 2;
      curv.m_attributes.m_name = "trimming curve";
    }
    else {
      ROS_WARNING("Produced invalid B-spline surface");
      return;
    }

    if(toggleWritingToFile)
    {
      std::string path = "/home/redbaron/catkin_ws/src/pmd_camboard_nano/samples/temp/";
      ss << path << "cloud_mesh_generator_nurbs_" << fileIdx << ".stl";
      pcl::io::savePolygonFileSTL(ss.str(), *mesh);
      ROS_INFO_STREAM("Writing to file \"" << ss.str() << "\"");
      fileIdx++;
      ss.str("");
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "cloud_mesh_generator_nurbs");
  ros::NodeHandle nh("~");
  std::string topicIn = "points_sor";
  bool toggleWriteToFile;

  // Surface fitting parameters
  int order, refinement, iterations_sf, mesh_res;
  double interior_smoothness, interior_weight, boundary_smoothness, boundary_weight;

  // Curve fitting parameters
  int cps_iterations, cps_max, iterations_cf, cp_res;
  double cps_accuracy, accuracy, cp_weight, cp_sigma2, interior_sigma2, smooth_concavity, smoothness;

  nh.param("write_to_file", toggleWriteToFile, false);

  // TODO Add setters to the CloudProcessingNodeMGNURBS class
  nh.param("order", order, 3);
  nh.param("refinement", refinement, 5);
  nh.param("iterations_sf", iterations_sf, 10);
  nh.param("mesh_res", mesh_res, 256);
  nh.param("interior_smoothness", interior_smoothness, .2);
  nh.param("interior_weight", interior_weight, 1.);
  nh.param("boundary_smoothness", boundary_smoothness, .2);
  nh.param("boundary_weight", boundary_weight, 0.);

  nh.param("cps_accuracy", cps_accuracy, 5e-2);
  nh.param("cps_iterations", cps_iterations, 3);
  nh.param("cps_max", cps_max, 200);
  nh.param("accuracy", accuracy, 1e-3);
  nh.param("iterations_cf", iterations_cf, 100);
  nh.param("cp_res", cp_res, 0);
  nh.param("cp_weight", cp_weight, 1.);
  nh.param("cp_sigma2", cp_sigma2, .1);
  nh.param("interior_sigma2", interior_sigma2, .00001);
  nh.param("smooth_concavity", smooth_concavity, 1.);
  nh.param("smoothness", smoothness, 1.);

  CloudProcessingNodeMGNURBS c(topicIn);
  ROS_INFO_STREAM("Writing to files " << (toggleWriteToFile ? "activated" : "deactivated"));
  c.setWritingToFile(toggleWriteToFile);
  //ROS_INFO_STREAM((polynomialFit ? "Enabling" : "Disabling") << " polynomial fit and setting search radius to " << searchRadius);
  // ...

  while(nh.ok())
    ros::spin();

  return 0;
}

