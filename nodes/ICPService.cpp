#include <iostream>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>


#include "icp_service/ICPTransform.h"

typedef pcl::PointXYZ ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> RGBCloud;
typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;


boost::shared_ptr<pcl::visualization::PCLVisualizer> View1 ()  {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


bool findICPTransform (icp_service::ICPTransform::Request &req,
		       icp_service::ICPTransform::Response &res) {


  pcl::PCLPointCloud2 cloud_in_PCLP2, cloud_out_PCLP2;
  ColorCloud::Ptr cloud_in(new ColorCloud);
  ColorCloud::Ptr cloud_out(new ColorCloud);

  //sensor_msgs::PointCloud2 pc1 = req.pc1, pc2 = req.pc2;
  pcl_conversions::toPCL(req.pc1, cloud_in_PCLP2);
  pcl_conversions::toPCL(req.pc2, cloud_out_PCLP2);

  pcl::fromPCLPointCloud2(cloud_in_PCLP2, *cloud_in);
  pcl::fromPCLPointCloud2(cloud_out_PCLP2, *cloud_out);

  /**
  pcl::VoxelGrid<ColorCloud> sor1,sor2;
  sor1.setInputCloud (cloud_in);
  sor1.setLeafSize (0.005f, 0.005f, 0.005f);
  sor1.filter (*cloud_in);

  sor2.setInputCloud (cloud_out);
  sor2.setLeafSize (0.005f, 0.005f, 0.005f);
  sor2.filter (*cloud_out);
  **/

  //boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer = View1();
  //viewer->addPointCloud<pcl::PointXYZ> (cloud_in, "cloud_in");
  //viewer->addPointCloud<pcl::PointXYZ> (cloud_out, "cloud_out");
  //while (!viewer->wasStopped ())
  //viewer->spinOnce (100);

    
  Eigen::Affine3d guess_tfm, found_tfm;
  tf::poseMsgToEigen(req.guess,guess_tfm);

  Eigen::Matrix4f icp_guess;
  icp_guess.block(0,0,3,3) = guess_tfm.linear().cast<float>().transpose();
  icp_guess.block(0,3,3,1) = guess_tfm.translation().cast<float>();
  

  pcl::GeneralizedIterativeClosestPoint<ColorPoint, ColorPoint> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);


  //boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  //icp.setTransformationEstimation(point_to_plane);

  icp.setMaximumIterations(500);
  icp.setRANSACIterations(20);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(0.05);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-4);

  std::cout<<"Max iterations: "<<icp.getMaximumIterations ()<<std::endl;
  std::cout<<"Max RANSAC iterations: "<<icp.getRANSACIterations ()<<std::endl;
  std::cout<<"RANSAC Outlier Rejection Threshold: "<< icp.getRANSACOutlierRejectionThreshold ()<<std::endl;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  std::cout<<"Max correspondence distance: "<<icp.getMaxCorrespondenceDistance ()<<std::endl;
  // Set the transformation epsilon (criterion 2)
  std::cout<<"Transformation epsilon: "<< icp.getTransformationEpsilon ()<<std::endl;
  // Set the euclidean distance difference epsilon (criterion 3)
  std::cout<<"Euclidean fitness epsilon: "<< icp.getEuclideanFitnessEpsilon ()<<std::endl;

  std::cout<<"PC1 size: "<<cloud_in->points.size()<<std::endl;
  std::cout<<"PC2 size: "<<cloud_out->points.size()<<std::endl;

  ColorCloud Final;
  icp.align(Final, icp_guess);

  std::cout << "Converged:" << icp.hasConverged() << ", Score: " << icp.getFitnessScore() << std::endl;
  
  Eigen::Matrix4f icpTfm = icp.getFinalTransformation();
  found_tfm.linear() = icpTfm.cast<double>().transpose().block(0,0,3,3);
  found_tfm.translation() = icpTfm.cast<double>().block(0,3,3,1);;

  tf::poseEigenToMsg(found_tfm, res.pose);
  return true; 
}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "icp_transform");
  ros::NodeHandle n;
  ros::ServiceServer icpService =
    n.advertiseService ("icpTransform", findICPTransform);

  std::cout<<"Spawned ICP service. Ready for request."<<std::endl;

  ros::Duration(1.0).sleep();
  ros::spin ();

  return 0;
}
