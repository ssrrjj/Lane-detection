#include "polyline.h"
#include <pcl/filters/voxel_grid.h>
#include <set>

using namespace cv;
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


//
void show(CloudPtr cloud, vector<pcl::PointXYZ> points) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p : cloud->points)
        tmp->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
    viewer->addPointCloud<pcl::PointXYZ>(tmp, "cloud");
    for (int point_idx = 0; point_idx < points.size() - 1; point_idx++) {
        viewer->addLine(points[point_idx], points[point_idx + 1], 255, 0, 0, "line" + to_string(point_idx));
    }
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
bool point_sorter(pcl::PointXYZ const& p1, pcl::PointXYZ const& p2) {
    return p1.x < p2.x;
}
void ransac_line(CloudPtr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients) {


    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    seg.setMaxIterations(1000);
    seg.setNumberOfThreads(8);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_LMEDS);
    seg.setDistanceThreshold(0.2); // pcfitplane use 0.05 as the maxDistance, here use 0.15, need some calibration

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

}


PolyLine::PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    vector<float> xlimit = getXLimits(cloud);
    if (xlimit[1] - xlimit[0] < 20)
        return;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.15f, 0.15f, 0.15f);
    sor.filter(*cloud_filtered);
    cout << cloud_filtered->points.size() << endl;
    
    vector<pcl::PointXYZ> pts;
    for (auto point : cloud_filtered->points) {
        pts.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    sort(pts.begin(), pts.end(), &point_sorter);
    cout << pts[0].x << endl;
    CloudPtr to_start = filterByField(cloud_filtered, "x", pts[0].x, pts[0].x + 5);


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    ransac_line(to_start, inliers, coefficients);
    Vec3f N(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    Vec3f X0(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    N = N / N[0];
    Vec3f X1 = X0 + N * (pts[0].x - X0[0]);
    Vec3f X2 = X0 + N * (pts[0].x + 5 - X0[0]);
    points.push_back(pcl::PointXYZ(X1[0], X1[1], X1[2]));
    points.push_back(pcl::PointXYZ(X2[0], X2[1], X2[2]));
    int left_idx = to_start->points.size();
    int right_idx = left_idx;
    float left_x = pts[0].x + 5;
    Vec2f last_dir(N[0], N[1]);
    Vec2f last_p(X2[0], X2[1]);
    cout << left_x << " " << pts[left_idx].x << endl;
    CloudPtr toshow = filterByField(cloud, "x", pts[0].x, left_x);
    cout << "get show cloud" << endl;
    show(toshow, points);
    //show(toshow, points);

    set<int> grid;
    while (true) {
        int i = left_idx;
        
        grid.clear();
        while (grid.size() < 25 && i < pts.size()) {
            grid.insert((int)(pts[i].x / 0.1));
            i += 1;
        }
        
        right_idx = i;
        int theta;
        Vec2f best_dir;
        int most_inlier = 0;
        vector<int> best_inliers;
        vector<int> cur_inliers;

        last_p = Vec2f(points.back().x, points.back().y);
        for (theta = -10; theta < 11; theta++) {
            float rad = M_PI / 180 * theta;
            float cos_theta = cos(rad), sin_theta = sin(rad);
            Vec2f new_dir(cos_theta * last_dir[0] - sin_theta * last_dir[1], sin_theta * last_dir[0] + cos_theta * last_dir[1]);
            float a = new_dir[1], b = -new_dir[0], c = new_dir[0] * last_p[1] - new_dir[1] * last_p[0];
            //new_dir = last_dir;



            cur_inliers.clear();
            for (i = left_idx; i < right_idx; i++) {

                pcl::PointXYZ cur_p = pts[i];
                float dis = abs(a * cur_p.x + b * cur_p.y + c) / sqrt(a * a + b * b);
                if (theta > 100)
                    cout << dis << endl;
                if (dis < 0.1)
                    cur_inliers.push_back(i);
            }
            if (cur_inliers.size() > most_inlier) {
                best_inliers = cur_inliers;
                most_inlier = cur_inliers.size();
                best_dir = new_dir;
            }


            //show cloud and line: 
            if (theta > 100) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
                CloudPtr tmp2 = filterByField(cloud_filtered, "x", left_x, left_x + 3);
                for (i = left_idx; i < right_idx; i++)
                    tmp->points.push_back(pcl::PointXYZ(pts[i].x, pts[i].y, pts[i].z));
                tmp->height = 1;
                tmp->width = tmp->points.size();
                pcl::PointXYZ leftp = points.back();
                pcl::PointXYZ rightp(leftp.x + 3 * new_dir[0], leftp.y + 3 * new_dir[1], leftp.z);
                pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
                viewer->setBackgroundColor(0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZ>(tmp, "cloud");
                viewer->addLine(leftp, rightp, 255, 0, 0, "line");
                while (!viewer->wasStopped())
                {
                    viewer->spinOnce(100);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        
        cur_inliers.clear();
        cout << most_inlier << " " << best_inliers.size() << endl;
        if (most_inlier == 0) {
            
            if (pts.back().x-left_x<5)
                break;
            i = left_idx;
            grid.clear();
            while (grid.size() < 50 && i < pts.size()) {
                grid.insert((int)(pts[i].x / 0.1));
                i += 1;
            }
            right_idx = i;
            i -= 1;
            cout << "left_idx, right_idx:" << left_idx <<" " << i << " "<<pts.size()<<endl;
            to_start = filterByField(cloud_filtered, "x", pts[left_idx].x, pts[i].x);
            cout << "to_start size:" << to_start->points.size() << endl;


            ransac_line(to_start, inliers, coefficients);
            N = Vec3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
            X0 = Vec3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

            N = N / N[0];
            Vec3f X1 = X0 + N * (pts[left_idx].x - X0[0]);
            Vec3f X2 = X0 + N * (pts[i].x - X0[0]);
            //points.push_back(pcl::PointXYZ(X1[0], X1[1], X1[2]));
            points.push_back(pcl::PointXYZ(X2[0], X2[1], X2[2]));
            //show(to_start, points);
            last_dir[0] = N[0];
            last_dir[1] = N[1];
            left_idx = right_idx;
            left_x = pts[i].x;
        }
        else {
            last_dir = best_dir;
            left_idx = right_idx;
            int inlier_idx = best_inliers[most_inlier - 1];
            points.push_back(pcl::PointXYZ(pts[inlier_idx].x, pts[inlier_idx].y, pts[inlier_idx].z));

            left_x = pts[inlier_idx].x;

            
        }
        toshow = filterByField(cloud, "x", pts[0].x, left_x);

        cout << "get show cloud" << endl;
        show(toshow, points);
        if (right_idx >= pts.size() - 1)
            break;
    }
    //show(toshow, points);
    //show(cloud, points);
}

PolyLine::PolyLine() {}