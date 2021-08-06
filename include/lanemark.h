#ifndef INCLUDE_LANEMARK_H_
#define INCLUDE_LANEMARK_H_
#include "utils.h"

cv::Vec4f getLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>& idx);
cv::Vec6f getLine3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>& idx);

class Element {
public: 
	int idx;
	int rank;
	Element* p;
};
class LaneMark: public Element  {
public:

	pcl::PointCloud<pcl::PointXYZI>::Ptr points;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	int max_i;
	int min_i;
	float minx, miny;
	float maxx, maxy;
	
	cv::Vec4f min_line, max_line;
	LaneMark(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int k, float search_radius = 15) {
		points = cloud;
		idx = k;
		p = this;
		
		maxx = cloud->points[0].x;
		minx = cloud->points[0].x;
		
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
		


		kdtree.setInputCloud(cloud);
		vector<int>min_neighbor, max_neighbor;
		vector<float>max_dis, min_dis;
		kdtree.radiusSearch(cloud->points[min_i], search_radius, min_neighbor, min_dis);
		kdtree.radiusSearch(cloud->points[max_i], search_radius, max_neighbor, max_dis);
		min_line = getLine(cloud, min_neighbor);
		max_line = getLine(cloud, max_neighbor);
		

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
bool is_coline(LaneMark* x, LaneMark* y, float th = 10.0);

bool is_coline(LaneMark3D* x, LaneMark3D* y, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float th = 0.75);

// void group(vector<LaneMark*>& input, vector<LaneMark*>& output);
#endif