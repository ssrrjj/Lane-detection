#include "lanedetection.h"
#include "open3d/Open3D.h"
using namespace std;
int main(int argc, char* argv[])
{
	string pcdfile;
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	if (!open3d::io::ReadPointCloud("ptcROI_debug.pcd", *cloud_ptr)) {
		cout << "unable to open pcd" << endl;
		return 0; 
	}
	//open3d::visualization::DrawGeometries({ cloud_ptr });
	std::tuple<Eigen::Vector4d, std::vector<size_t>>plane;
	plane = cloud_ptr->SegmentPlane(0.05, 3, 1000);
	Eigen::Vector4d plane_model = get<0>(plane);
	std::vector<size_t> inliers = get<1>(plane);
	auto inlier_cloud = std::make_shared<open3d::geometry::PointCloud>();
	inlier_cloud = cloud_ptr->SelectByIndex(inliers, FALSE);
	open3d::visualization::DrawGeometries({ inlier_cloud });
	return (0);

}