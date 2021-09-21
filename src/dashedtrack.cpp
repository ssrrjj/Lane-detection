#include "utils.h"
#include "polyline.h"
#include "lanedetection.h"
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <chrono>
#include <thread>
using namespace std;
using namespace pcl;

float norm(cv::Vec2f v) {
	return sqrt(v[0] * v[0] + v[1] * v[1]);
}
float norm(cv::Vec3f v) {
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

PointXYZI pointxyzi(cv::Vec3f p) {
	return PointXYZI(p[0], p[1], p[2]);
}

PointXYZ pointxyz(cv::Vec3f p) {
	return PointXYZ(p[0], p[1], p[2]);
}

PointXYZI shift(PointXYZI p, cv::Vec3f v) {
	p.x += v[0];
	p.y += v[1];
	p.z += v[2];
	return p;
}

cv::Vec3f vec(const PointXYZI& p) {
	return cv::Vec3f(p.x, p.y, p.z);
}

cv::Vec3f vec(const PointXYZ& p) {
	return cv::Vec3f(p.x, p.y, p.z);
}

void showlines(CloudPtr cloud, vector<pcl::PointXYZ> points) {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto p : cloud->points)
		tmp->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
	viewer->addPointCloud<pcl::PointXYZ>(tmp, "cloud");
	for (int point_idx = 0; point_idx < points.size() - 1; point_idx+=2) {
		if (point_idx == points.size() - 2) {
			viewer->addLine(points[point_idx], points[point_idx + 1], 255, 0, 0, "line" + to_string(point_idx));
		}
		else
			viewer->addLine(points[point_idx], points[point_idx + 1], 0, 255, 0, "line" + to_string(point_idx));
	}
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

PolyLine dashedtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, cv::Vec3f p1, cv::Vec3f p2, cv::Vec3f p3) {
	
	cout << "begin track" << endl;
	PolyLine ret = PolyLine();
	ret.points.push_back(pointxyz(p1));
	ret.points.push_back(pointxyz(p2));
	//ret.points.push_back(pointxyz(p3));

	CloudPtr subcloud;

	cv::Vec3f dir1 = p2-p1;
	cv::Vec3f dir2 = p3-p2;
	float stripe_len = norm(dir1);
	float stripe_gap = norm(dir2);
	cout << "stripe_len:" << stripe_len << endl;
	cout << "stripe_gap:" << stripe_gap << endl;
	dir1 /= stripe_len;
	dir2 /= stripe_gap;
	float mid_d = stripe_gap + stripe_len / 2;

	cv::Vec3f last_dir = (dir1 + dir2) / 2.0;
	last_dir /= norm(last_dir);

	cv::Vec3f tmp = p3 + last_dir * stripe_len / 2;

	
	PointXYZI cur_mid(tmp[0], tmp[1], tmp[2]);
	
	CloudPtr toshow(new PointCloud<PointXYZI>);
	while (true) {
		vector<int> indset;
		Eigen::Vector4d plane_model;
		std::vector<float> pointRadiusSquaredDistance;
		std::vector<int> pointIdxRadiusSearch;
		kdtree->radiusSearch(cur_mid, stripe_len/2 + min(1.0f, stripe_gap/2), pointIdxRadiusSearch,
			pointRadiusSquaredDistance);
		cout << "sub cloud size " << pointIdxRadiusSearch.size() << endl;

		if (pointIdxRadiusSearch.size() < 4) {
			//ret.points.push_back(PointXYZ(cur_mid.x, cur_mid.y, cur_mid.z));
			break;

		}
		subcloud = select(cloud, pointIdxRadiusSearch);

		float distThreshold = 0.1;

		pcfitplaneByROI(subcloud, indset, plane_model, distThreshold);

		cout << "get plane" << endl;



		CloudPtr plane = select(subcloud, indset);
		//custom_pcshow(plane);
		
		//project cur_mid to the plane:
		cv::Vec3f normal(plane_model[0], plane_model[1], plane_model[2]);
		cout << norm(normal) << endl;
		cur_mid = shift(cur_mid, (-normal[0] * cur_mid.x - normal[1] * cur_mid.y - normal[2] * cur_mid.z - plane_model[3]) * normal);

		
		vector<vector<int>> pixel2cloud;
		vector<int> lane_mark_idx;
		float img_x, img_y;
		cv::Mat uimage = toImage(plane, plane_model, 0.1, pixel2cloud);
		cout << uimage.cols << " " << uimage.rows << endl;
		/*cv::imshow("image", uimage);
		cv::waitKey(0);*/

		cv::Mat lanemark = findLaneInImage(uimage);
		//lanemark = removeFalsePostive(lanemark, )
		/*cv::imshow("lanemark", lanemark);
		cv::waitKey(0);*/

		int h = uimage.rows;
		int w = uimage.cols;
		lanemark = removeFalsePostive(lanemark, LanePar());
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
		PointXYZI best_mid;
		PointXYZI cur;
		cv::Vec2f vert_dir(-last_dir[1], last_dir[0]);
		vert_dir /= norm(vert_dir);
		vert_dir *= 0.02;

		CloudPtr cloud_2d(new PointCloud<PointXYZI>);
		for (auto& point : lane_mark_cloud->points) {
			cloud_2d->points.push_back(PointXYZI(point.x, point.y, 0));
		}
		cloud_2d->height = 1;
		cloud_2d->width = cloud_2d->points.size();

		vector<PointXYZ> showline;
		cv::Vec2f best_dir(last_dir[0], last_dir[1]);
		vector<PointXYZ> candidate_lines;
		for (int divert = -20; divert < 21; divert++) {
			cur.x = cur_mid.x + divert * vert_dir[0];
			cur.y = cur_mid.y + divert * vert_dir[1];
			cur.z = cur_mid.z;

			for (int theta = -10; theta < 11; theta++) {
				float rad = M_PI / 180 * theta;
				float cos_theta = cos(rad), sin_theta = sin(rad);
				cv::Vec2f new_dir(cos_theta * last_dir[0] - sin_theta * last_dir[1], sin_theta * last_dir[0] + cos_theta * last_dir[1]);
				float a = new_dir[1], b = -new_dir[0], c = new_dir[0] * cur.y - new_dir[1] * cur.x;
				//new_dir = last_dir;



				cur_inliers.clear();
				for (int i = 0; i < lane_mark_cloud->points.size(); i++) {
					PointXYZI cur_p = lane_mark_cloud->points[i];
					//cout << (cur_p.x - cur.x) * last_dir[0] + (cur_p.y - cur.y) * last_dir[1] << endl;
					
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
					best_mid = cur;
				}
				//cout <<"cur inliers:"<< cur_inliers.size() << endl;
				if (theta == 0) {
					candidate_lines.push_back(PointXYZ(cur.x - 2 * new_dir[0], cur.y - 2 * new_dir[1], 0));
					candidate_lines.push_back(PointXYZ(cur.x + 2 * new_dir[0], cur.y + 2 * new_dir[1], 0));
				}
				//show(cloud_2d, showline);
			}
		}
		cout << "inliers:" << best_inliers.size() << " theta " << best_theta << endl;
		if (best_inliers.size() < 4)
			break;
		cur_inliers.clear();
		cur = best_mid;
		float d_min= 0, d_max = 0;
		best_dir /= norm(best_dir);

		last_dir[0] = best_dir[0];
		last_dir[1] = best_dir[1];
		last_dir[2] = 0;
		last_dir = last_dir - (last_dir[0] * normal[0]+last_dir[1]*normal[1]+last_dir[2]*normal[2]) * normal;
		last_dir /= norm(last_dir);

		for (int i : best_inliers) {
			PointXYZI cur_p = lane_mark_cloud->points[i];
			float cur_d = (cur_p.x - cur.x) * last_dir[0] + (cur_p.y - cur.y) * last_dir[1] + (cur_p.z - cur.z) * last_dir[2];
			if (cur_d > d_max) {
				d_max = cur_d;
			}
			if (cur_d < d_min) {
				d_min = cur_d;
			}
		}
		//cout << "dmin:" << d_min << " dmax:" << d_max << endl;

		PointXYZI max_p = shift(cur, d_max * last_dir);
		PointXYZI min_p = shift(cur, d_min * last_dir);
		
		float cur_stripe_len = norm(vec(max_p) - vec(min_p));
		float cur_stripe_gap = norm(vec(min_p) - vec(ret.points.back()));
		cout << "cur stripe_len:" << cur_stripe_len << endl;
		cout << "cur stripe_gap:" << cur_stripe_gap << endl;
		stripe_len += 0.1 * (cur_stripe_len - stripe_len);
		stripe_gap += 0.1 * (cur_stripe_gap - stripe_gap);

		cur_mid = shift(max_p, last_dir * (stripe_gap + 0.5 * stripe_len));
		cout << "show" << endl;
		*toshow += *lane_mark_cloud;
		//ret.points.clear();
		candidate_lines.push_back(PointXYZ(min_p.x, min_p.y, 0));
		candidate_lines.push_back(PointXYZ(max_p.x, max_p.y, 0));
		//showlines(cloud_2d, candidate_lines);
		ret.points.push_back(PointXYZ(min_p.x, min_p.y, min_p.z));
		ret.points.push_back(PointXYZ(max_p.x, max_p.y, max_p.z));
		//show(toshow, ret.points);
	}
	//show(toshow, ret.points);

	ofstream myfile("line.txt");
	for (int i = 0; i < 1; i++) {
		//writer.write<pcl::PointXYZI>("lanemark_" + to_string(i) + ".pcd", *all_marks[i]->points3D, false);


		//showpolyline(all_marks[i]->points3D, polyline.points);
		string line = to_string(i) + " ";
		for (auto& point : ret.points) {
			line += to_string(point.x) + ",";
			line += to_string(point.y) + ",";
			line += to_string(point.z) + " ";

		}
		myfile << line;
	}
	myfile.close();

	return ret;
}
