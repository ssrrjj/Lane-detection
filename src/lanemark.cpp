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

float gety(cv::Vec4f& line, float x) {
	return line[1] / line[0] * (x - line[2]) + line[3];
}

bool is_coline(LaneMark* x, LaneMark* y, float th, int v, float * score) {
	LaneMark* lm1 = x;
	LaneMark* lm2 = y;
	bool ret = false;
	if (x->maxx > y->maxx) {
		lm1 = y;
		lm2 = x;
	}
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree1;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree2;
	kdtree1.setInputCloud(lm1->points);
	kdtree2.setInputCloud(lm2->points);
	vector<float>pointRadiusSquaredDistance;
	vector<int>pointIdxRadiusSearch;
	int nn1 = kdtree1.radiusSearch(pcl::PointXYZI(lm2->minx, lm2->miny, 0), 5, pointIdxRadiusSearch,
		pointRadiusSquaredDistance);
	int nn2 = kdtree2.radiusSearch(pcl::PointXYZI(lm1->maxx, lm1->maxy, 0), 5, pointIdxRadiusSearch,
		pointRadiusSquaredDistance);
	
	
	float midx = (lm1->maxx + lm2->minx) / 2;
	float midy = (lm1->maxy + lm2->miny) / 2;
	float dx = lm1->maxx - lm2->minx;
	float dy = lm1->maxy - lm2->miny;
	//cout << "dis:" << sqrt(dx * dx + dy * dy) << endl;
	float vx1 = lm1->max_line[0], vy1 = lm1->max_line[1];
	float vx2 = lm2->min_line[0], vy2 = lm2->min_line[1];
	//cout << "angle:" << abs(vx1 * vx2 + vy1 * vy2) << endl;
	if (VERBOSE == 2 && v == 1) {
		cout << lm1->maxx << " " << lm2->minx << endl;
		cout << nn1 << " " << nn2 << endl;
		cout << "dis:" << dx * dx + dy * dy << endl;
		cout << "angle:" << abs(vx1 * vx2 + vy1 * vy2) << endl;
		pcl::PointXYZ l1p1, l1p2, l2p1, l2p2;
		l1p1.x = lm1->maxx;
		l1p1.y = gety(lm1->max_line, l1p1.x);
		l1p2.x = midx;
		l1p2.y = gety(lm1->max_line, l1p2.x);

		l2p1.x = midx;
		l2p1.y = gety(lm2->min_line, l2p1.x);
		l2p2.x = lm2->minx;
		l2p2.y = gety(lm2->min_line, l2p2.x);

		custom_pcshow(lm1->points, lm2->points, l1p1, l1p2, l2p1, l2p2);

	}

	cv::Vec4f& line1 = lm1->max_line;
	cv::Vec4f& line2 = lm2->min_line;

	//cout << "get dis" << endl;
	float d1 = disFromPointToLine(midx, midy, line1);
	float d2 = disFromPointToLine(midx, midy, line2);
	if (score != nullptr)
		*score = sqrt(dx * dx + dy * dy) + d1 + d2;

	//cout << d1 + d2 << endl;
	if (nn1) {
		//cout << "nn close true" << endl;
		return true;
	}
	if (nn2) {
		//cout << "nn close true" << endl;
		return true;
	}
	if (dx * dx + dy * dy < 50) {
		//cout << "close true" << endl;
		return true;
	}
	//cout << "is too far away" << dx * dx + dy * dy << endl;
	if (dx * dx + dy * dy > 250000 && v == 1) {
		//cout << "too far away false" << endl;
		return false;
	}
	/*if (lm1->maxx > lm2->minx)
		return false;*/

	if (abs(vx1 * vx2 + vy1 * vy2) < 0.9) {
		//cout << "angle false" << endl;
		return false;

	}
	if (d1 + d2 < th) {

		//cout << "dir true" << endl;
		return true;
	}
	//cout << "final false" << endl;
	return false;
	
	//cout << d1 + d2 << endl;
	
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
	//custom_pcshow(cloud, lm1->points, lm2->points, line1, line2);
	th = 0.3;
	if (d1 + d2 < th) {
		//cout << d1 + d2 << endl;
		custom_pcshow(cloud, lm1->points, lm2->points, line1, line2);
		return true;
	}
	return false;

}

vector<LaneMark*>  group2(vector<LaneMark*>& lanemarks) {
	vector<LaneMark*> ret;
	
	for (LaneMark* lanemark : lanemarks) {
		LaneMark* p = (LaneMark*)findp(lanemark);
		if (p == lanemark) {
			ret.push_back(p);
		}
		else {
			if (lanemark->minx < p->minx) {
				p->minx = lanemark->minx;
				p->miny = lanemark->miny;
				p->min_line = lanemark->min_line;
			}
			if (lanemark->maxx > p->maxx) {
				p->maxx = lanemark->maxx;
				p->maxy = lanemark->maxy;
				p->max_line = lanemark->max_line;
			}
			*p->points += *lanemark->points;
			*p->points3D += *lanemark->points3D;
			delete lanemark;
		}
	}
	return ret;
}

vector<LaneMark*>  group(vector<LaneMark*>& lanemarks) {
	vector<LaneMark*> ret;
	for (int i = 0; i < lanemarks.size(); i++) {
		for (int j = i + 1; j < lanemarks.size(); j++) {
			if (findp(lanemarks[i]) == findp(lanemarks[j]))
				continue;
			if (is_coline(lanemarks[i], lanemarks[j])) {

				join(lanemarks[i], lanemarks[j]);
				//cout << "join" << i << " " << j << " " << findp(lanemarks[i])->idx << " " << findp(lanemarks[j])->idx << endl;
			}
		}
	}
	//cout << "group coline marks" << endl;
	for (LaneMark* lanemark : lanemarks) {
		LaneMark* p = (LaneMark*)findp(lanemark);
		if (p == lanemark) {
			ret.push_back(p);
		}
		else {
			if (lanemark->minx < p->minx) {
				p->minx = lanemark->minx;
				p->miny = lanemark->miny;
				p->min_line = lanemark->min_line;
			}
			if (lanemark->maxx > p->maxx) {
				p->maxx = lanemark->maxx;
				p->maxy = lanemark->maxy;
				p->max_line = lanemark->max_line;
			}
			*p->points += *lanemark->points;
			*p->points3D += *lanemark->points3D;
			delete lanemark;
		}
	}
	return ret;
}

void
custom_pcshow(vector<LaneMark*> & marks) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    vector<cv::Vec3b> colors;
    for (int i = 0; i < marks.size(); i++)
    {
		//cout << i << endl;
        int b = cv::theRNG().uniform(0, 256);
        int g = cv::theRNG().uniform(0, 256);
        int r = cv::theRNG().uniform(0, 256);
        colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    int k = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto mark : marks) {
		//cout << k << endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        toRGB(mark->points3D, rgb, colors[k]);
		//cout << "get rgb" << endl;
		*whole += *rgb;
		k += 1;
    }
	viewer->addPointCloud<pcl::PointXYZRGB>(whole, "whole");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}