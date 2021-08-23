#include "lanedetection.h"
#include "utils.h"
#include <filesystem>
#include "LasStream.h"

namespace fs = std::filesystem;




int main(int argc, char* argv[])
{

	//cv::Mat uimage = cv::imread(argv[1], cv::IMREAD_ANYDEPTH);
	string input = argv[1];
	string output = argv[2];
    const fs::path input_path(argv[1]);
	cout << input << " " << output << endl;
    for (const auto& entry : fs::directory_iterator(input_path)) {
		
		fs::path entry_path = entry.path();
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
		if (entry_path.extension() == ".pcd") {
			pcl::io::loadPCDFile(input+"/"+entry_path.filename().string(), *cloud);
		}
		else if (entry_path.extension() == ".las") {
			readlas(entry_path.filename().string(), cloud);
		}
		else
			continue;
		cout << entry_path.filename().string() << endl;
		string name = entry_path.stem().string();
		float subregion_width = 50;
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
		int N = (range[1] - range[0] + 1) / subregion_width;

		vector<float> roi(2, 0.0);
		for (int i = 0; i <= N; i++) {
			roi[0] = range[0] + subregion_width * i;
			roi[1] = range[0] + subregion_width * (i + 1);

			if (i == N) roi[1] = range[1];
			std::cout << "INFO: process points in range " << fieldname << ": " << roi[0] << ", " << roi[1] << std::endl;
			vector<int> indsetCloud;
			pcl::PointCloud<pcl::PointXYZI>::Ptr ptcROI = filterByField(cloud, indsetCloud, fieldname, roi[0], roi[1]);
			
			vector<int> indsetROI;
			Eigen::Vector4d plane_model;
			if (ptcROI->points.size() < 100)
				continue;
			pcfitplaneByROI(ptcROI, indsetROI, plane_model, 0.1, fieldname);
			vector<vector<int>> pixel2cloud;

			pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = select(ptcROI, indsetROI);
			cv::Mat uimage = toImage(planeCloud, plane_model, 0.1, pixel2cloud);
			//cout << "toImage" << endl;
			vector<int>cloud2pixel;
			vector<vector<int>> clusters = ImageDbscan(uimage, cloud2pixel, 4, 10);
			//cout << "ImageDbscan" << endl;
			int numClusters = clusters.size();
			//cout << "get clusters " << clusters.size() << endl;
			int max_cluster_size = 0, max_cluster_idx = 0;
			int h = uimage.rows, w = uimage.cols;
		
			if (numClusters == 0) {
				cout << "WARNING: Point density is too low.\n" << endl;
				continue;
			}
			for (int i = 0; i < numClusters; i++) {
				if (clusters[i].size() > max_cluster_size) {
					max_cluster_size = clusters[i].size();
					max_cluster_idx = i;
				}
			}

			cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);
			cv::Mat mask_inv = cv::Mat::ones(h, w, CV_8UC1);
			int mini = h, maxi = 0, minj = w, maxj = 0;
			//cout << "compute mask" << endl;
			for (int point_idx : clusters[max_cluster_idx]) {
				int pixel_idx = cloud2pixel[point_idx];
				int i = pixel_idx / w;
				int j = pixel_idx % w;
				mini = min(i, mini);
				maxi = max(i, maxi);
				minj = min(j, minj);
				maxj = max(j, maxj);
				mask.at<uchar>(i, j) = 1;
				mask_inv.at<uchar>(i, j) = 0;
			}
			//cout << "get image" << endl;
			//cv::imshow("mask", mask);
			//cv::waitKey(0);
			// mask uimage to get road area image
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < w; j++) {
					uimage.at<uchar>(i, j) *= mask.at<uchar>(i, j);
				}
			}
			//cout << "mask image" << endl;
			static cv::Mat lookUpTable(1, 256, CV_8U);
			static bool compute_table = TRUE;
			float gamma_ = 0.7;
			if (compute_table) {
				uchar* p = lookUpTable.ptr();
				for (int i = 0; i < 256; ++i)
					p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
				compute_table = FALSE;
			}
			
			cv::Mat gamma_img = cv::Mat::zeros(h, w, CV_8UC1);
			cv::LUT(uimage, lookUpTable, gamma_img);
			
			//cv::imshow("road_image_gamma", gamma_img);
			//cv::waitKey(0);
			
			cv::Mat subimage(uimage, cv::Rect(minj, mini, maxj - minj + 1, maxi - mini + 1));
			
			//cv::imshow("sub", uimage);
			//cv::waitKey(0);
			cv::imwrite(output + "/" + name + "_" + to_string(i) + ".png", subimage);
		}
    }
	return 0;

}
