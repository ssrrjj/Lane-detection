#include "lanemark.h"
#include <cmath>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

void link(Element* x, Element* y) {
	if (x == y)
		return;
	if (x->rank > y->rank)
		y->p = x;
	else
		x->p = y;
	if (x->rank == y->rank)
		y->rank += 1;
	return;
}

Element* findp(Element* x) {
	if (x->p != x)
		x->p = findp(x->p);
	return x->p;
}

void join(Element* x, Element* y) {
	Element* xp = findp(x);
	Element* yp = findp(y);
	link(xp, yp);
}

cv::Vec4f getLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>& idx) {
	vector<cv::Point2f> line_pts;
	for (int i : idx) {
		cv::Point2f tmp;
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		line_pts.push_back(tmp);
	}
	
	cv::Vec4f line;
	cv::fitLine(line_pts, line, cv::DIST_HUBER, 0, 0.1, 0.1);

	//vector<float> xlimit = getXLimits(cloud);
	//vector<float> ylimit = getYLimits(cloud);
	//int w = int(xlimit[1]) + 10;
	//int h = int(ylimit[1]) + 10;
	//cv::Mat visual = cv::Mat::zeros(h, w, CV_8UC3);
	//for (int i : idx) {
	//	int pi = (int)cloud->points[i].y;
	//	int pj = (int)cloud->points[i].x;
	//	visual.at<cv::Vec3b>(pi, pj) = cv::Vec3b(255, 255, 255);
	//}

	//float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
	//float y1 = vy / vx * (xlimit[0] - x0) + y0;
	//float y2 = vy / vx * (xlimit[1] - x0) + y0;

	//cv::line(visual, cv::Point(xlimit[0], y1), cv::Point(xlimit[1], y2), cv::Scalar(255, 0, 0));
	//cv::imshow("mark", visual);
	//cv::waitKey(0);
	return line;

}

cv::Vec6f getLine3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>& idx) {
	
	pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_s(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(select(cloud, idx)));
	pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_s);
	ransac.setDistanceThreshold(.01);
	ransac.computeModel();
	cv::Vec6f ret(ransac.model_coefficients_[3], ransac.model_coefficients_[4], ransac.model_coefficients_[5], ransac.model_coefficients_[0], ransac.model_coefficients_[1], ransac.model_coefficients_[2]);
	return ret;

	vector<cv::Point3f> line_pts;
	for (int i : idx) {
		cv::Point3f tmp;
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		line_pts.push_back(tmp);
	}

	cv::Vec6f line;
	cv::fitLine(line_pts, line, cv::DIST_HUBER, 0, 0.01, 0.01);

	return line;

}

float disFromPointToLine(float x, float y, cv::Vec4f line) {
	float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
	float d = abs(vy * x - vx * y + vx * y0 - vy * x0);
	return d;
}

bool is_coline(LaneMark* x, LaneMark* y, float th) {
	LaneMark* lm1 = x;
	LaneMark* lm2 = y;
	if (x->maxx > y->minx) {
		lm1 = y;
		lm2 = x;
	}
	if (lm1->maxx > lm2->minx)
		return false;


	cv::Vec4f& line1 = lm1->max_line;
	cv::Vec4f& line2 = lm2->min_line;
	float midx = (lm1->maxx + lm2->minx) / 2;
	float midy = (lm1->maxy + lm2->miny) / 2;
	
	float d1 = disFromPointToLine(midx, midy, line1);
	float d2 = disFromPointToLine(midx, midy, line2);

	//cout << d1 + d2 << endl;
	
	if (d1 + d2 < th) {
		cout << lm1->idx << " " << lm2->idx << endl;

		//custom_pcshow(lm1->points);
		//custom_pcshow(lm2->points);
		int w = max(1+lm1->maxx, 1+lm2->maxx);
		vector<float> ylimit1 = getYLimits(lm1->points);
		vector<float> ylimit2 = getYLimits(lm2->points);
		int h = (int)max(ylimit1[1], ylimit2[1]) + 1;
		cv::Mat visual = cv::Mat::zeros(h, w , CV_8UC1);
		cout << h << " " << w << endl;
		for (auto p : lm1->points->points) {
			int pi = (int)p.y;
			int pj = (int)p.x;
			assert(pi < h);
			assert(pj < w);
			visual.at<uchar>(pi, pj) = 255;
		}
		for (auto p : lm2->points->points) {
			int pi = p.y;
			int pj = p.x;
			assert(pi < h);
			assert(pj < w);
			visual.at<uchar>(pi, pj) = 255;
		}
		/*cv::line(visual, cv::Point(0, -line1[2] * line1[1] / line1[0] + line1[3]), cv::Point(midx, line1[1] / line1[0] * (midx - line1[2]) + line1[3]), 100, 10);

		cv::line(visual, cv::Point(0, -line2[2] * line2[1] / line2[0] + line2[3]), cv::Point(midx, line2[1] / line2[0] * (midx - line2[2]) + line2[3]), 100, 10);*/
		cv::imshow("two lines", visual);
		cv::waitKey(0);
		return true;
	}
	return false;

}

float disFromPointToLine(float x, float y, float z, cv::Vec6f line) {
	float vx = line[0], vy = line[1], vz = line[2];
	float x0 = line[3], y0 = line[4], z0 = line[5];
	float norm2 = (x0-x)*(x0-x)+(y0-y)*(y0-y)+(z0-z)*(z0-z);
	float dotp = (x0 - x) * vx + (y0 - y) * vy + (z0 - z) * vz;
	float d = sqrt(norm2 - dotp*dotp);
	//d = abs(vy * x - vx * y + vx * y0 - vy * x0);
	//cout << x << "," << y << "," << z << "," << line << endl;
	return d;
}
bool is_coline(LaneMark3D* x, LaneMark3D* y, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float th) {
	
	LaneMark3D* lm1 = x;
	LaneMark3D* lm2 = y;
	if (x->endpoint1.x > y->endpoint0.y) {
		lm1 = y;
		lm2 = x;
	}
	cv::Vec6f& line1 = lm1->line1;
	cv::Vec6f& line2 = lm2->line0;
	cv::Point3f point1 = lm1->endpoint1;
	cv::Point3f point2 = lm2->endpoint0;
	float midx = (point1.x + point2.x) / 2;
	float midy = (point1.y + point2.y) / 2;
	float midz = (point1.z + point2.z) / 2;

	float d1 = disFromPointToLine(midx, midy, midz, line1);
	float d2 = disFromPointToLine(midx, midy, midz, line2);

	//cout << d1 + d2 << endl;

	if (d1 + d2 < th) {
		cout << d1 + d2 << endl;
		//custom_pcshow(cloud, lm1->points, lm2->points, line1, line2);
		return true;
	}
	return false;

}