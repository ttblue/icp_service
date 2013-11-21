#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

#include "icp_service/ICPTransform.h"

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;

bool findICPTransform (icp_service::ICPTransform::Request &req,
			 icp_service::ICPTransform::Response &res) {

  ColorCloud::Ptr cloud_in(new ColorCloud);
  ColorCloud::Ptr cloud_out(new ColorCloud);
  pcl::fromROSMsg(req.pc1, *cloud_in);
  pcl::fromROSMsg(req.pc2, *cloud_out);

  pcl::IterativeClosestPoint<ColorPoint, ColorPoint> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  ColorCloud Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

  Eigen::Matrix4f icpTfm = icp.getFinalTransformation();
  Eigen::Affine3d tfm;
  tfm.linear() = icpTfm.cast<double>().transpose().block(0,0,3,3);
  tfm.translation() = icpTfm.cast<double>().block(0,3,3,1);;

  tf::poseEigenToMsg(tfm, res.pose);
  return icp.hasConverged();
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
