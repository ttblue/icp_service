#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;

bool ICPTransform(ColorCloud::Ptr cloud_in, 
		  ColorCloud::Ptr cloud_out,
		  Eigen::Affine3d guess_tfm,
		  Eigen::Affine3d &found_tfm);
