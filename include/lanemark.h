#ifndef INCLUDE_LANEMARK_H_
#define INCLUDE_LANEMARK_H_
#include "utils.h"

#include <boost/make_shared.hpp>
#include "polyline.h"
#include <pcl/filters/voxel_grid.h>
cv::Vec4f getLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>& idx);
cv::Vec6f getLine3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>& idx);

class Element {
public: 
	int idx;
	double rank;
	Element* p;
};
class LaneMark: public Element  {
public:
	static int global_idx;
	pcl::PointCloud<pcl::PointXYZI>::Ptr points;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	
	float minx, miny;
	float maxx, maxy;
	pcl::PointCloud<pcl::PointXYZI>::Ptr points3D;
	cv::Vec4f min_line, max_line;
	LaneMark(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<vector<int>>& pixel2cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr whole, int w, float search_radius = 80) {
		points = cloud;
		idx = global_idx;
		global_idx += 1;
		p = this;
		int max_i = 0;
		int min_i = 0;
		maxx = cloud->points[0].x;
		minx = cloud->points[0].x;
		rank = maxx;
		for (int i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].x < minx) {
				minx = cloud->points[i].x;
				min_i = i;
			}
			if (cloud->points[i].x > maxx) {
				maxx = cloud->points[i].x;
				max_i = i;
			}
		}
		maxy = cloud->points[max_i].y;
		miny = cloud->points[min_i].y;
		


		
		vector<int>min_neighbor, max_neighbor;
		vector<float>max_dis, min_dis;
		
		pcl::VoxelGrid<pcl::PointXYZI> sor;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.1f, 0.1f, 0.1f);
		sor.filter(*cloud_filtered);
		kdtree.setInputCloud(cloud_filtered);
		kdtree.radiusSearch(cloud->points[min_i], search_radius, min_neighbor, min_dis);
		kdtree.radiusSearch(cloud->points[max_i], search_radius, max_neighbor, max_dis);


		min_line = getLine(cloud_filtered, min_neighbor);
		max_line = getLine(cloud_filtered, max_neighbor);
		#if PCL_VERSION_COMPARE(<,1,11,0)
			
		    points3D = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
		#else
		    points3D = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
		#endif
		for (auto& point : cloud->points) {
			int img_j = (int)point.x;
			int img_i = (int)point.y;
			//cout << img_i << "," << img_j << endl;
			for (int idx3D : pixel2cloud[img_i * w + img_j]) {
				points3D->points.push_back(whole->points[idx3D]);
			}
		}
		points3D->height = 1;
		points3D->width = points3D->points.size();
		/*int h = (int)max(maxy, miny) + 20;
		cv::Mat visual = cv::Mat::zeros(h, (int)maxx + 20, CV_8UC1);
		cout << h << " " << (int)maxx + 20 << endl;
		custom_pcshow(cloud);
		for (int i = 0; i < cloud->points.size(); i++) {
			int pi = cloud->points[i].y;
			int pj = cloud->points[i].x;
			cout << pi << " " << pj << " " << endl;
			visual.at<uchar>(pi, pj) = 255;
		}
		int tt = 1;
		if (min_line[0] < 0)
			tt = -1;
		cv::line(visual, cv::Point(minx, miny), cv::Point(minx - 10 * tt * min_line[0], miny - 10 * tt * min_line[1]), 100);

		tt = 1;
		if (max_line[0] < 0)
			tt = -1;
		cv::line(visual, cv::Point(maxx, maxy), cv::Point(maxx + 10 * tt * max_line[0], maxy + 10 * tt * max_line[1]), 100);
		cv::imshow("create lanemark profile", visual);
		cv::waitKey(0);*/
	}
	
};

class LaneMark3D : public Element {
public:

	pcl::PointCloud<pcl::PointXYZI>::Ptr points;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	cv::Point3f endpoint0;
	cv::Point3f endpoint1;

	cv::Vec6f line0, line1;
	

	LaneMark3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int k) {
		//cout << "constructor ";
		points = cloud;
		idx = k;
		
		p = this;

		float maxx = cloud->points[0].x;
		float minx = cloud->points[0].x;
		int min_i = 0, max_i = 0;
		for (int i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].x < minx) {
				minx = cloud->points[i].x;
				min_i = i;
			}
			if (cloud->points[i].x > maxx) {
				maxx = cloud->points[i].x;
				max_i = i;
			}
		}
		endpoint0 = cv::Point3f(minx, cloud->points[min_i].y, cloud->points[min_i].z);
		endpoint1 = cv::Point3f(maxx, cloud->points[max_i].y, cloud->points[max_i].z);
		
		float search_radius = 1.5;
		kdtree.setInputCloud(cloud);
		vector<int>min_neighbor, max_neighbor;
		vector<float>max_dis, min_dis;
		kdtree.radiusSearch(cloud->points[min_i], search_radius, min_neighbor, min_dis);
		kdtree.radiusSearch(cloud->points[max_i], search_radius, max_neighbor, max_dis);
		//cout << "get neighbors ";
		line0 = getLine3D(cloud, min_neighbor);
		line1 = getLine3D(cloud, max_neighbor);

		//cout << "get lines " << endl;
		/*int h = (int)max(maxy, miny) + 20;
		cv::Mat visual = cv::Mat::zeros(h, (int)maxx + 20, CV_8UC1);
		cout << h << " " << (int)maxx + 20 << endl;
		custom_pcshow(cloud);
		for (int i = 0; i < cloud->points.size(); i++) {
			int pi = cloud->points[i].y;
			int pj = cloud->points[i].x;
			cout << pi << " " << pj << " " << endl;
			visual.at<uchar>(pi, pj) = 255;
		}
		int tt = 1;
		if (min_line[0] < 0)
			tt = -1;
		cv::line(visual, cv::Point(minx, miny), cv::Point(minx - 10 * tt * min_line[0], miny - 10 * tt * min_line[1]), 100);

		tt = 1;
		if (max_line[0] < 0)
			tt = -1;
		cv::line(visual, cv::Point(maxx, maxy), cv::Point(maxx + 10 * tt * max_line[0], maxy + 10 * tt * max_line[1]), 100);
		cv::imshow("create lanemark profile", visual);
		cv::waitKey(0);*/
	}

};
void link(Element* x, Element* y);
Element* findp(Element* x);
void join(Element* x, Element* y);
bool is_coline(LaneMark* x, LaneMark* y, float th = 10.0, int v = 0, float*score = nullptr);

bool is_coline(LaneMark3D* x, LaneMark3D* y, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float th = 0.75);
vector<LaneMark*>  group(vector<LaneMark*>& lanemarks);
// void group(vector<LaneMark*>& input, vector<LaneMark*>& output);
void custom_pcshow(vector<LaneMark*>& marks);
vector<LaneMark*>  group2(vector<LaneMark*>& lanemarks);
#endif
