#include "utils.h"
#include "polyline.h"
#include "lanedetection.h"
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <string>
#include <shapefil.h>
#include <vector>
#include "LasStream.h"
using namespace std;
using namespace pcl;

PolyLine solidtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, PointXYZ p1, PointXYZ p2) {
	//cout << "begin track" << endl;
	PolyLine ret = PolyLine();
	ret.points.push_back(p1);
	ret.points.push_back(p2);
	
	CloudPtr subcloud;

	cv::Vec2f last_dir(p2.x - p1.x, p2.y - p1.y);
	PointXYZI cur(p2.x,p2.y,p2.z,0);
	CloudPtr toshow(new PointCloud<PointXYZI>);
	while (true) {
		vector<int> indset;
		Eigen::Vector4d plane_model;
		std::vector<float> pointRadiusSquaredDistance;
		std::vector<int> pointIdxRadiusSearch;
		kdtree->radiusSearch(cur, 1.5, pointIdxRadiusSearch,
			pointRadiusSquaredDistance);
		//cout << "sub cloud size " << pointIdxRadiusSearch.size() << endl;
		if (pointIdxRadiusSearch.size() < 4)
			break;
		subcloud = select(cloud, pointIdxRadiusSearch);

		float distThreshold = 0.1;

		pcfitplaneByROI(subcloud, indset, plane_model, distThreshold);

		//cout << "get plane" << endl;
		CloudPtr plane = select(subcloud, indset);
		//custom_pcshow(plane);
		vector<vector<int>> pixel2cloud;
		vector<int> lane_mark_idx;
		float img_x, img_y;
		cv::Mat uimage = toImage(plane, plane_model, 0.1, pixel2cloud);
		//cout << uimage.cols << " " << uimage.rows << endl;
		/*cv::imshow("image", uimage);
		cv::waitKey(0);*/

		cv::Mat lanemark = findLaneInImage(uimage);
		/*cv::imshow("lanemark", lanemark);
		cv::waitKey(0);*/

		int h = uimage.rows;
		int w = uimage.cols;

		lane_mark_idx.clear();
		for (int i = 0; i < h; i++) {
			for (int j = 0; j < w; j++) {
				// find the points corresponding to pixel at [i,j]
				if (lanemark.at<uchar>(i, j) > 0) {
					for (int corres_idx : pixel2cloud[i * w + j]) {
						lane_mark_idx.push_back(corres_idx);
					}
				}
			}
		}
		CloudPtr lane_mark_cloud = select(plane, lane_mark_idx);
		//custom_pcshow(lane_mark_cloud);

		//downsample

		//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
		//pcl::VoxelGrid<pcl::PointXYZI> sor;
		//sor.setInputCloud(lane_mark_cloud);
		//sor.setLeafSize(0.05f, 0.05f, 0.05f);
		//sor.filter(*cloud_filtered);
		//cloud = cloud_filtered;
		//cout << "downsample" << endl;

		vector<int>cur_inliers;
		vector<int>best_inliers;
		double most_inlier = 0;
		int best_theta = 0;
		cv::Vec2f best_dir = last_dir;
		for (int theta = -20; theta < 21; theta++) {
			float rad = M_PI / 360 * theta;
			float cos_theta = cos(rad), sin_theta = sin(rad);
			cv::Vec2f new_dir(cos_theta * last_dir[0] - sin_theta * last_dir[1], sin_theta * last_dir[0] + cos_theta * last_dir[1]);
			float a = new_dir[1], b = -new_dir[0], c = new_dir[0] * cur.y - new_dir[1] * cur.x;
			//new_dir = last_dir;



			cur_inliers.clear();
			for (int i = 0; i < lane_mark_cloud->points.size();i ++) {
				PointXYZI cur_p = lane_mark_cloud->points[i];
				//cout << (cur_p.x - cur.x) * last_dir[0] + (cur_p.y - cur.y) * last_dir[1] << endl;
				if ((cur_p.x - cur.x) * last_dir[0] + (cur_p.y - cur.y) * last_dir[1] < 0)
					continue;
				float dis = abs(a * cur_p.x + b * cur_p.y + c) / sqrt(a * a + b * b);
				if (dis < 0.05)
					cur_inliers.push_back(i);
			}
			//cout << theta << " " << exp(-theta * theta / 150.0) << endl;
			if (cur_inliers.size() * exp(-theta * theta / 150.0) > most_inlier) {
				best_inliers = cur_inliers;
				most_inlier = cur_inliers.size() * exp(-theta * theta / 150.0);
				best_dir = new_dir;
				best_theta = theta;
			}
		}
		//cout <<"inliers:"<< best_inliers.size() << " theta "<<best_theta<<endl;
		if (best_inliers.size() <4)
			break;
		cur_inliers.clear();
		PointXYZI next;
		float farthest_d = 0;
		for (int i : best_inliers) {
			PointXYZI cur_p = lane_mark_cloud->points[i];
			float cur_d = (cur_p.x - cur.x) * best_dir[0] + (cur_p.y - cur.y) * best_dir[1];
			if (cur_d > farthest_d) {
				farthest_d = cur_d;
				next.x = cur_p.x;
				next.y = cur_p.y;
				next.z = cur_p.z;
			}
		}
		last_dir = best_dir;
		cur = next;
		//cout << "show" << endl;
		*toshow += *lane_mark_cloud;
		
		ret.points.push_back(PointXYZ(next.x,next.y,next.z));
	}
	ret.cuts.push_back(ret.points.size());
	//show(toshow, ret.points);

	//ofstream myfile("line.txt");
	//for (int i = 0; i < 1; i++) {
	//	//writer.write<pcl::PointXYZI>("lanemark_" + to_string(i) + ".pcd", *all_marks[i]->points3D, false);
	//	

	//	//showpolyline(all_marks[i]->points3D, polyline.points);
	//	string line = to_string(i) + " ";
	//	for (auto& point : ret.points) {
	//		line += to_string(point.x) + ",";
	//		line += to_string(point.y) + ",";
	//		line += to_string(point.z) + " ";

	//	}
	//	myfile << line;
	//}
	//myfile.close();

	return ret;
}

int solidtrack(CloudPtr cloud, vector<float> p1, vector<float> p2, string save_file) {

	pcl::PointXYZ pclp1(p1[0], p1[1], p1[2]);
	pcl::PointXYZ pclp2(p2[0], p2[1], p2[2]);
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
	kdtree->setInputCloud(cloud);
	PolyLine polyline = solidtrack(cloud, kdtree, pclp1, pclp2);
	

	SHPHandle shp = SHPCreate(save_file.c_str(), SHPT_ARCZ);
	SHPClose(shp);
	shp = SHPOpen(save_file.c_str(), "r+b");
	int n_vertices = 0;
	vector<double>shp_x, shp_y, shp_z;
	vector<int>panstart;

	for (int cutidx = 0; cutidx < polyline.cuts.size(); cutidx++) {

		panstart.push_back(shp_x.size());
		for (int i = (cutidx == 0) ? 0 : polyline.cuts[cutidx - 1]; i < polyline.cuts[cutidx]; i++) {
			auto& point = polyline.points[i];
			shp_x.push_back(point.x);
			shp_y.push_back(point.y);
				
			shp_z.push_back(point.z);
		}
	}
	//cout << shp_x.size() << endl;
	//for (auto panstarti : panstart)
	//	cout << panstarti << " ";
	//cout << endl;
	double* x_array = new double[shp_x.size()];
	double* y_array = new double[shp_x.size()];
	double* z_array = new double[shp_x.size()];
	int* panstart_array = new int[panstart.size()];
	memcpy(x_array, shp_x.data(), shp_x.size() * sizeof(double));
	memcpy(y_array, shp_y.data(), shp_x.size() * sizeof(double));
	memcpy(z_array, shp_z.data(), shp_x.size() * sizeof(double));
	memcpy(panstart_array, panstart.data(), panstart.size() * sizeof(int));
	SHPObject* obj = SHPCreateObject(SHPT_ARCZ, 2, panstart.size(), panstart_array, NULL, shp_x.size(), x_array, y_array, z_array, NULL);
	SHPWriteObject(shp, -1, obj);
	SHPDestroyObject(obj);
	SHPClose(shp);
	delete[]x_array;
	delete[]y_array;
	delete[]z_array;
	delete[]panstart_array;

	return 0;
}