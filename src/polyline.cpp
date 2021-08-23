#include "polyline.h"
string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

pcl::PointXYZ centroid(vector < pcl::PointXYZI>points) {
    pcl::PointXYZ ret(0, 0, 0);
    for (auto point : points) {
        ret.x += point.x/points.size();
        ret.y += point.y/points.size();
        ret.z += point.z/points.size();
    }
    return ret;
}
//
PolyLine::PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
	//int n_pts = cloud->points.size();
//	cv::Mat pca_data = cv::Mat(n_pts, 2, CV_32F);
//	for (int i = 0; i < n_pts; i++) {
//		pca_data.at<float>(i, 0) = cloud->points[i].x;
//		pca_data.at<float>(i, 1) = cloud->points[i].y;
//	}
//	cv::PCA pca_analysis(pca_data, cv::Mat(), cv::PCA::DATA_AS_ROW);
//    Eigen::Matrix4f T;
//    T.row(0) = Eigen::Vector4f(pca_analysis.eigenvectors.at<float>(0, 0), pca_analysis.eigenvectors.at<float>(0, 1), 0., 0.);
//    T.row(1) = Eigen::Vector4f(pca_analysis.eigenvectors.at<float>(1, 0), pca_analysis.eigenvectors.at<float>(1, 1), 0., 0.);
//    T.row(0) = Eigen::Vector4f(1., 0., 0., 0.);
//    T.row(1) = Eigen::Vector4f(0., 1., 0., 0.);
//    T.row(2) = Eigen::Vector4f(0.,0., 1., 0.);
//    T.row(3) = Eigen::Vector4f(0., 0., 0., 1.);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::transformPointCloud(*cloud, *transformed_cloud, T);
//    vector<float> xlimit = getXLimits(transformed_cloud);
//    float step = 0.5;
//
//    int bins = (int)((xlimit[1] - xlimit[0]) / step) + 1;
//    vector<vector<pcl::PointXYZI>> bins_of_points(bins);
//    
//    for (auto & point : transformed_cloud->points) {
//        int bin_idx = (point.x - xlimit[0]) / step;
//        assert(bin_idx >= 0 && bin_idx < bins);
//        bins_of_points[bin_idx].push_back(point);
//    }
//    pcl::PointXYZI leftend(transformed_cloud->points[0]), rightend(transformed_cloud->points[0]);
//
//    for (auto& point : transformed_cloud->points) {
//        if (point.x < leftend.x)
//            leftend = point;
//        if (point.x > rightend.x)
//            rightend = point;
//    }
//    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
//
//    tmp->points.push_back(pcl::PointXYZ(leftend.x, leftend.y, leftend.z));
//    
//
//    for (int i = 0; i < bins; i++) {
//        if (bins_of_points[i].size() > 0) {
//            tmp->points.push_back(centroid(bins_of_points[i]));
//        }
//    }
//    tmp->points.push_back(pcl::PointXYZ(rightend.x, rightend.y, rightend.z));
//    tmp->height = 1;
//    tmp->width = tmp->points.size();
//    
//    points = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//
//    pcl::transformPointCloud(*tmp, *points, T.inverse());

    //---------------------------------------------------------------
    /*int n_pts = cloud->points.size();
    cv::Mat points_array = cv::Mat(n_pts, 3, CV_32F);
    for (int i = 0; i < n_pts; i++) {
        points_array.at<float>(i, 0) = cloud->points[i].x;
        points_array.at<float>(i, 1) = cloud->points[i].y;
        points_array.at<float>(i, 2) = cloud->points[i].z;
    }
    cv::Vec6f line;
    cv::fitLine(points_array, line, cv::DIST_HUBER, 0, 0.1, 0.1);
    
    vector<float> xlimit = getXLimits(cloud);

    cv::Vec3f X0(line[3], line[4], line[5]);
    cv::Vec3f N(line[0], line[1], line[2]);
    if (N[0] < 0)
        N = -N;
    cv::Vec3f leftend = X0 + N * (xlimit[0] - X0[0]);
    cv::Vec3f rightend = X0 + N * (xlimit[1] - X0[0]);
    points = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    points->points.push_back(pcl::PointXYZ(leftend[0], leftend[1], leftend[2]));
    points->points.push_back(pcl::PointXYZ(rightend[0], rightend[1], rightend[2]));*/

    //---------------------------------------------------------
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_LMEDS);
    seg.setDistanceThreshold(0.05); // pcfitplane use 0.05 as the maxDistance, here use 0.15, need some calibration
    seg.setMaxIterations(1000);
    seg.setNumberOfThreads(8);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    //for (float k : coefficients->values)
    //    cout << k << " ";
    // cout << endl;
    vector<float> xlimit = getXLimits(cloud);
    vector<float>& line = coefficients->values;
    cv::Vec3f N(line[3], line[4], line[5]);
    cv::Vec3f X0(line[0], line[1], line[2]);
    
    N = N / N[0];
    cv::Vec3f leftend = X0 + N * (xlimit[0] - X0[0]);
    cv::Vec3f rightend = X0 + N * (xlimit[1] - X0[0]);
    points = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    points->points.push_back(pcl::PointXYZ(leftend[0], leftend[1], leftend[2]));
    points->points.push_back(pcl::PointXYZ(rightend[0], rightend[1], rightend[2]));

}