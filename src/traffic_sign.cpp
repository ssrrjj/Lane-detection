#include "utils.h"
#include "pcfitplane.h"
#include <pcl/filters/voxel_grid.h>
void FindTrafficSignByROI(CloudPtr cloud) {
	if (cloud->points.size() < 100)
		return;
	CloudPtr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	vector<int> plane_inliers;
	Eigen::Vector4d plane_model;

	// downsample point cloud for better plane fitting
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(*cloud_filtered);
	cout << cloud_filtered->points.size() << endl;
	//cloud = cloud_filtered;
	pcfitplaneByROI(cloud_filtered, plane_inliers, plane_model, 0.3);
	
	CloudPtr aboveplane(new pcl::PointCloud<pcl::PointXYZI>);

	vector<int> aboveplane_inliers;
	for (int i = 0; i < cloud->points.size(); i++) {
		float a, b, c, d;
		a = plane_model[0];
		b = plane_model[1];
		c = plane_model[2];
		d = plane_model[3];
		float x, y, z;
		x = cloud->points[i].x;
		y = cloud->points[i].y;
		z = cloud->points[i].z;
		if (abs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c) > 3) {
			aboveplane_inliers.push_back(i);
		}
	}
	cout << "above plane:" << aboveplane->points.size() << endl;
	custom_pcshow(aboveplane);
	aboveplane = select(cloud, aboveplane_inliers);
	vector<Eigen::Vector3f> eigenvalues;
	pca(aboveplane, eigenvalues);

	vector<int> sign_inliers;
	for (int i = 0; i < aboveplane->points.size(); i++) {
		Eigen::Vector3f & eigenvalue = eigenvalues[i];
		if (eigenvalue[0] == 0)
			continue;

		float lam1 = eigenvalue[0], lam2 = eigenvalue[1], lam3 = eigenvalue[2];
		if ((lam2 - lam3) / lam1)
			sign_inliers.push_back(i);

	}

	CloudPtr sign = select(aboveplane, sign_inliers);
	custom_pcshow(sign);
}