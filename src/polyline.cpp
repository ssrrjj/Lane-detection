#include "polyline.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
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
vector<pcl::PointXYZ> ransac_line(CloudPtr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients) {


    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setMaxIterations(1000);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_LMEDS);
    seg.setDistanceThreshold(0.2); 

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    //pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr
    //    model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud));
    //pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_l);
    //ransac.setDistanceThreshold(0.2);
    //ransac.computeModel();
    //ransac.getInliers(inliers->indices);
    //Eigen::Vector4d 
    //model_l->computeModelCoefficients(inliers->indices, coefficients->values)
    vector<pcl::PointXYZ> ret(2);
    bool first = true;
    cout << "ransac line inliers:" << inliers->indices.size() << endl;
    for (int i : inliers->indices) {
        
        pcl::PointXYZI tmp = cloud->points[i];
        pcl::PointXYZ t(tmp.x, tmp.y, tmp.z);
        if (first) {
            first = false;
            ret[0] = t;
            ret[1] = t;

        }
        else {
            if (t.x > ret[1].x)
                ret[1] = t;
            if (t.x < ret[0].x)
                ret[0] = t;
        }
    }
    cout << cloud->points.size() << endl;
    //custom_pcshow(cloud);
    //show(cloud, ret);
    return ret;
}


PolyLine::PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float length_threshold, bool downsample) {
    vector<float> xlimit = getXLimits(cloud);
    if (xlimit[1] - xlimit[0] < length_threshold)
        return;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    if (downsample) {
        int Nsegs = (xlimit[1] - xlimit[0] + 1) / 50;
        vector<float>roi(2);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        for (int i = 0; i <= Nsegs; i++) {
            roi[0] = xlimit[0] + 50 * i;
            roi[1] = xlimit[0] + 50 * (i + 1);
            if (i == Nsegs) roi[1] = xlimit[1];
            sub_cloud = filterByField(cloud, "x", roi[0], roi[1]);
            sor.setInputCloud(sub_cloud);
            sor.setLeafSize(0.15f, 0.15f, 0.15f);
            sor.filter(*sub_cloud_filtered);
            *cloud_filtered += *sub_cloud_filtered;
        }
    }
    else
        cloud_filtered = cloud;
    //sor.setInputCloud(cloud);
    //sor.setLeafSize(0.15f, 0.15f, 0.15f);
    //sor.filter(*cloud_filtered);
    
    //cout << "polyline" << endl;
    vector<pcl::PointXYZ> pts;
    for (auto point : cloud_filtered->points) {
        pts.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    sort(pts.begin(), pts.end(), &point_sorter);

    set<int> grid;

    int start_right = 0;
    grid.clear();
    while (grid.size() < 50 && start_right < pts.size()) {
        grid.insert((int)(pts[start_right].x / 0.1));
        start_right += 1;
    }
    start_right -= 1;
     
    CloudPtr to_start = filterByField(cloud_filtered, "x", pts[0].x, pts[start_right].x);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    vector<pcl::PointXYZ> endpts = ransac_line(to_start, inliers, coefficients);
    Vec3f N = Vec3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    Vec3f X0 = Vec3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    N = N / N[0];
    //Vec3f X1 = X0 + N * (pts[left_idx].x - X0[0]);
    //Vec3f X2 = X0 + N * (pts[i].x - X0[0]);
    //points.push_back(pcl::PointXYZ(X1[0], X1[1], X1[2]));
    points.push_back(endpts[1]);
    //show(to_start, points);
    Vec2f last_dir(N[0], N[1]);
    Vec2f last_p(endpts[1].x, endpts[1].y);
    int left_idx = start_right;
    int right_idx = start_right;
    float left_x = pts[right_idx].x;
    points.push_back(endpts[0]);
    points.push_back(endpts[1]);
    CloudPtr toshow = filterByField(cloud, "x", pts[0].x, left_x);
    cout << points.back().x << " " << points.back().y << endl;
    //show(toshow, points);
    //---------------------------------------------bad-----------------------------------------
    /*CloudPtr to_start = filterByField(cloud_filtered, "x", pts[0].x, pts[0].x + 3);


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    ransac_line(to_start, inliers, coefficients);
    Vec3f N(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    Vec3f X0(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    N = N / N[0];
    Vec3f X1 = X0 + N * (pts[0].x - X0[0]);
    Vec3f X2 = X0 + N * (pts[0].x + 3 - X0[0]);
    points.push_back(pcl::PointXYZ(X1[0], X1[1], X1[2]));
    points.push_back(pcl::PointXYZ(X2[0], X2[1], X2[2]));
    int left_idx = to_start->points.size();
    int right_idx = left_idx;
    float left_x = pts[0].x + 3;
    Vec2f last_dir(N[0], N[1]);
    Vec2f last_p(X2[0], X2[1]);
    CloudPtr toshow = filterByField(cloud, "x", pts[0].x, left_x);*/
    //show(toshow, points);

    
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
                if (dis < 0.1)
                    cur_inliers.push_back(i);
            }
            if (cur_inliers.size() > most_inlier) {
                best_inliers = cur_inliers;
                most_inlier = cur_inliers.size();
                best_dir = new_dir;
            }


            //show cloud and line: 
            /*if (theta > 0) {
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
            }*/
        }
        
        cur_inliers.clear();
        //cout << most_inlier << " " << best_inliers.size() << endl;
        if (most_inlier == 0) {
            
            if (pts.back().x-left_x<3)
                break;
            i = left_idx;
            grid.clear();
            while (grid.size() < 50 && i < pts.size()) {
                grid.insert((int)(pts[i].x / 0.1));
                i += 1;
            }
            right_idx = i;
            i -= 1;
            to_start = filterByField(cloud_filtered, "x", pts[left_idx].x, pts[i].x);


            vector<pcl::PointXYZ> endpts = ransac_line(to_start, inliers, coefficients);
            cout << coefficients->values[0] << " " << coefficients->values[2] << " " << coefficients->values[3] << " " << coefficients->values[4] << endl;
            N = Vec3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
            X0 = Vec3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

            N = N / N[0];
            //Vec3f X1 = X0 + N * (pts[left_idx].x - X0[0]);
            //Vec3f X2 = X0 + N * (pts[i].x - X0[0]);
            //points.push_back(pcl::PointXYZ(X1[0], X1[1], X1[2]));
            points.push_back(endpts[1]);
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
        cout << points.back().x <<" "<< points.back().y << endl;
        //show(toshow, points);
        if (right_idx >= pts.size() - 1)
            break;
    }
    //show(toshow, points);
    //show(cloud, points);
}

PolyLine::PolyLine() {}