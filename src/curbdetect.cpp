#include "curbdetect.h"
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <polyline.h>
Voxel::Voxel() {
	points = make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	points->height = 1;
	points->width = 0;
	max_z = 0;
	min_z = 0;
	
	
}
void Voxel::add(pcl::PointXYZI p, int i) {
	if (points->points.size() == 0) {
		max_z = p.z;
		min_z = p.z;
	}
	else {
		if (p.z > max_z)
			max_z = p.z;
		if (p.z < min_z)
			min_z = p.z;
	}
	points->points.push_back(p);
	points->width += 1;
	idx.push_back(i);
	return;
}
float Voxel::getnorm(Eigen::Vector4f& plane_parameters) {
	float curvature;
	pcl::computePointNormal(*points,plane_parameters,curvature);
	Eigen::Vector3f vert(0, 0, 1);
	Eigen::Vector3f norm_vec(plane_parameters[0], plane_parameters[1], plane_parameters[2]);
	
	norm_vec /= norm_vec.norm();
	return abs(norm_vec[2]);
}
void FindCurbByROI(CloudPtr cloud, CloudPtr result, CloudPtr flat_result) {
	cout << cloud->points.size() << endl;
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
	pcfitplaneByROI(cloud_filtered, plane_inliers, plane_model, 0.1);
	
	plane_inliers.clear();
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
		if (abs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c) < 0.3) {
			plane_inliers.push_back(i);
		}
	}
	//cout << plane_inliers.size() << endl;
	CloudPtr plane = select(cloud, plane_inliers);
	
	savepcd(plane, "plane.pcd");
	//custom_pcshow(plane);
	//CloudPtr plane = select(cloud, plane_inliers);
	
	//rotate the plane to be horizontal.
	Eigen::Vector3f plane_norm((float)plane_model[0], (float)plane_model[1], (float)plane_model[2]);
	plane_norm /= plane_norm.norm();
	if (plane_norm[2] < 0)
		plane_norm *= -1;

	Eigen::Vector3f z_unit(0, 0, 1);
	Eigen::Vector3f axis = plane_norm.cross(z_unit);
	Eigen::Matrix3f R = Eigen::AngleAxisf(acos(z_unit.dot(plane_norm)), axis).toRotationMatrix();
	Eigen::Matrix4f T;
	T.setIdentity();
	T.block<3, 3>(0, 0) = R;

	CloudPtr transformed_plane(new pcl::PointCloud<pcl::PointXYZI>);
	
	
	pcl::transformPointCloud(*plane, *transformed_plane, T);
	plane = transformed_plane;

	



	//estimate normals using open3d
	std::shared_ptr<open3d::geometry::PointCloud> o3dplane = pclToO3d(plane);
	open3d::geometry::KDTreeSearchParamHybrid kdparam(0.1, 30);
	o3dplane->EstimateNormals(kdparam);

	// estimate normals using pcl
	//pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	//ne.setInputCloud(plane);

	//// Create an empty kdtree representation, and pass it to the normal estimation object.
	//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	//ne.setSearchMethod(tree);

	//// Output datasets
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	//ne.setRadiusSearch(0.1);

	//// Compute the features
	//ne.compute(*cloud_normals);


	vector<float> xlimit = getXLimits(plane);
	vector<float> ylimit = getYLimits(plane);

	float grid_size = 0.1;
	int h, w;
	w = int((xlimit[1] - xlimit[0]) / grid_size) + 1;
	h = int((ylimit[1] - ylimit[0]) / grid_size) + 1;
	cout << "h, w " << h << "," << w << endl;
	
	vector<Voxel> grid(h * w + 1);
	//cout << h * w + 1 << endl;
	for (int i = 0; i < o3dplane->points_.size(); i++) {
		auto& point = o3dplane->points_[i];
		double x = point[0];
		double y = point[1];
		int point_w = (x - xlimit[0]) / grid_size;
		int point_h = (y - ylimit[0]) / grid_size;
		
		grid[point_h * w + point_w].add(pcl::PointXYZI(x,y,point[2]), i);
	}
	vector<int>curb_inliers;
	//CloudPtr result(new pcl::PointCloud<pcl::PointXYZI>);
	CloudPtr result_trans(new pcl::PointCloud<pcl::PointXYZI>);

	CloudPtr elevation_diff(new pcl::PointCloud<pcl::PointXYZI>);
	for (int i = 0; i < grid.size(); i ++) {
		auto voxel = grid[i];

		if (voxel.points->points.size() == 0)
			continue;
		if (voxel.max_z - voxel.min_z > 0.08) {
			for (int idx : voxel.idx) {
				elevation_diff->points.push_back(plane->points[idx]);
				if (abs(o3dplane->normals_[idx][2]) < 0.8) {
					result_trans->points.push_back(plane->points[idx]);
				}
			}
		}
	}
	elevation_diff->height = 1;
	elevation_diff->width = elevation_diff->points.size();
	//custom_pcshow(elevation_diff);
	result_trans->height = 1;
	result_trans->width = result_trans->points.size();
	
	//result = select(plane, curb_inliers);
	// Rotate the result back
	
	pcl::transformPointCloud(*result_trans, *result, T.inverse());
	//custom_pcshow(result);
	//calculate planarity 

	vector<Eigen::Vector3f> eigen_values;
	pca(result, eigen_values);

	vector<int> planar_inliers;

	for (int i = 0; i < result->points.size(); i++) {
		Eigen::Vector3f& eigenvalue = eigen_values[i];
		if (eigenvalue[0] == 0)
			continue;

		float lam1 = eigenvalue[0], lam2 = eigenvalue[1], lam3 = eigenvalue[2];
		result->points[i].intensity = lam3 / (lam1 + lam2 + lam3);
	}
	
	//CloudPtr planar_result = select(result, planar_inliers);
	//custom_pcshow(planar_result);

	for (int i = 0; i < result->points.size(); i++) {
		float a, b, c, d;
		a = plane_model[0];
		b = plane_model[1];
		c = plane_model[2];
		d = plane_model[3];
		float x, y, z;
		x = result->points[i].x;
		y = result->points[i].y;
		z = result->points[i].z;
		float k = a * x + b * y + c * z + d;
		Eigen::Vector3f result_point(x, y, z);
		Eigen::Vector3f normal_vector(a, b, c);
		float normal_vector_norm = normal_vector.norm();
		Eigen::Vector3f projection = result_point - k / normal_vector_norm / normal_vector_norm * normal_vector;
		//cout << a * projection[0] + b * projection[1] + c * projection[2] + d << endl;
		flat_result->points.push_back(pcl::PointXYZI(projection[0], projection[1], projection[2], result->points[i].intensity));
	}
	flat_result->height = 1;
	flat_result->width = flat_result->points.size();
	savepcd(result, "result.pcd");
	
	return;
}

CloudPtr FindCurb(CloudPtr cloud) {
	float subregion_width = 30;
	string fieldname = "x";

	vector<float> range;
	vector<float> xlimits = getXLimits(cloud);
	vector<float> ylimits = getYLimits(cloud);
	vector<float> ilimits = getILimits(cloud);
	for (int i = 0; i < cloud->points.size(); i++) {
		cloud->points[i].intensity /= ilimits[1];
	}
	ilimits = getILimits(cloud);
	cout << "ilimits " << ilimits[1] << endl;
	std::cout << "X limit:" << xlimits[0] << " " << xlimits[1] << std::endl;
	std::cout << "Y limit:" << ylimits[0] << " " << ylimits[1] << std::endl;
	if (xlimits[1] - xlimits[0] < ylimits[1] - ylimits[0]) {
		fieldname = "y";
		range = ylimits;
	}
	else {
		fieldname = "x";
		range = xlimits;
	}
	Eigen::Matrix4d T;
	if (fieldname == "y") {
		
		T.row(0) = Eigen::Vector4d(0., 1., 0., 0.);
		T.row(1) = Eigen::Vector4d(1., 0., 0., 0.);
		T.row(2) = Eigen::Vector4d(0., 0., 1., 0.);
		T.row(3) = Eigen::Vector4d(0., 0., 0., 1.);
		pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::transformPointCloud(*cloud, *transformed_cloud, T);
		cloud = transformed_cloud;

	}

	int N = (range[1] - range[0] + 1) / subregion_width;

	vector<float> roi(2, 0.0);
	assert(par.start <= N);
	pcl::PCDWriter writer;
	CloudPtr result(new pcl::PointCloud<pcl::PointXYZI>);
	CloudPtr flat_result(new pcl::PointCloud<pcl::PointXYZI>);
	for (int i = 3; i <= N; i++) {
		roi[0] = range[0] + subregion_width * i;
		roi[1] = range[0] + subregion_width * (i + 1);
		if (i == N) roi[1] = range[1];
		CloudPtr subresult(new pcl::PointCloud<pcl::PointXYZI>);
		CloudPtr sub_flat(new pcl::PointCloud<pcl::PointXYZI>);
		CloudPtr subcloud = filterByField(cloud, "x", roi[0], roi[1]);
		FindCurbByROI(subcloud, subresult, sub_flat);
		*result += *subresult;
		*flat_result += *sub_flat;
		cout << "result:" << result->points.size() << endl;
		//custom_pcshow(result);
	}

	if (fieldname == "y") {
		for (auto& point : result->points) {
			float tmp = point.x;
			point.x = point.y;
			point.y = tmp;
		}
		for (auto& point : flat_result->points) {
			float tmp = point.x;
			point.x = point.y;
			point.y = tmp;
		}
	}

	// clustering
	DBSCAN dbscan;
	dbscan.setInputCloud(flat_result);
	std::vector<int> dbclustering = dbscan.segment(0.2, 3);
	std::vector<std::vector<int>>& clusters = dbscan.clusters;

	ofstream myfile("curb.txt");
	vector<int> cluster_inliers;
	int cluster_idx = 0;
	for (auto& cluster : clusters) {
		cluster_idx++;
		cout << cluster.size() << endl;
		
		float avg = 0;
		for (int in_cluster : cluster) {
			avg += flat_result->points[in_cluster].intensity;
		}
		avg /= cluster.size();
		if (avg > 0 && cluster.size() > 50) {
			cluster_inliers.insert(cluster_inliers.end(), cluster.begin(), cluster.end());
			CloudPtr sub_curb = select(result, cluster);
			PolyLine polyline(sub_curb, 5, false);
			//custom_pcshow(sub_curb);
			if (polyline.points.size() == 0)
				continue;
			
			string line = to_string(cluster_idx) + " ";
			for (auto& point : polyline.points) {
				line += to_string(point.x) + ",";
				line += to_string(point.y) + ",";
				line += to_string(point.z) + " ";

			}
			line += "\n";
			myfile << line;


		}
		
	}
	
	myfile.close();


	result = select(result, cluster_inliers);
	
	


	cout << "complete" << endl;
	cout << result->points.size() << endl;



	savepcd(result, "result.pcd");
	
	return result;
	
}
