#include <iostream>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

#include "pcl_icp.h"
#include "icp_service/ICPTransform.h"

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;

bool findICPTransform (icp_service::ICPTransform::Request &req,
			 icp_service::ICPTransform::Response &res) {


  ColorCloud::Ptr cloud_in(new ColorCloud);
  ColorCloud::Ptr cloud_out(new ColorCloud);
  pcl::fromROSMsg(req.pc1, *cloud_in);
  pcl::fromROSMsg(req.pc2, *cloud_out);

  Eigen::Affine3d guess_tfm, found_tfm;
  tf::poseMsgToEigen(req.guess,guess_tfm);

  bool succ = ICPTransform (cloud_in, cloud_out, guess_tfm, found_tfm); 

  tf::poseEigenToMsg(found_tfm, res.pose);
  return succ;
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
