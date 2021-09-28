/*
 * pcshow.cpp
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */

#include "pcshow.h"
#include <thread>
#include <chrono>


int user_data;

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "Once per viewer loop: " << count++;
//    viewer.removeShape ("text", 0);
//    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);
    viewer->addPointCloud<pcl::PointXYZI>(cloud);
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    // viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud);
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    // viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void toRGB(pcl::PointCloud<pcl::PointXYZI>::Ptr gray, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb, cv::Vec3b color) {

    for (int i = 0; i < gray->points.size(); i++) {
        pcl::PointXYZI tmp = gray->points[i];
        //cout << "get grey point " << i << "/"<< gray->points.size()<<endl;
        rgb->points.push_back(pcl::PointXYZRGB(tmp.x, tmp.y, tmp.z, color[0], color[1], color[2]));
    }
    rgb->height = 1;
    rgb->width = rgb->points.size();
    return;
}

void 
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr whole, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, cv::Vec6f line1, cv::Vec6f line2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb2(new pcl::PointCloud<pcl::PointXYZRGB>);

    toRGB(cloud1, rgb1, cv::Vec3b(255, 0, 0));
    toRGB(cloud2, rgb2, cv::Vec3b(0, 255, 0));
    if (whole)
        viewer->addPointCloud<pcl::PointXYZI>(whole, "whole");
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb1, "cloud1");
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb2, "cloud2");
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    // viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread(viewerPsycho);

    pcl::ModelCoefficients line_coeff1;
    line_coeff1.values.resize(6);    // We need 6 values
    line_coeff1.values[0] = line1[3];
    line_coeff1.values[1] = line1[4];
    line_coeff1.values[2] = line1[5];

    line_coeff1.values[3] = line1[0];
    line_coeff1.values[4] = line1[1];
    line_coeff1.values[5] = line1[2];

    pcl::ModelCoefficients line_coeff2;
    line_coeff2.values.resize(6);    // We need 6 values
    line_coeff2.values[0] = line2[3];
    line_coeff2.values[1] = line2[4];
    line_coeff2.values[2] = line2[5];

    line_coeff2.values[3] = line2[0];
    line_coeff2.values[4] = line2[1];
    line_coeff2.values[5] = line2[2];

    viewer->addLine(line_coeff1, "line1");
    viewer->addLine(line_coeff2, "line2");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void
custom_pcshow( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, pcl::PointXYZ l1p1, pcl::PointXYZ l1p2, pcl::PointXYZ l2p1, pcl::PointXYZ l2p2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb2(new pcl::PointCloud<pcl::PointXYZRGB>);

    toRGB(cloud1, rgb1, cv::Vec3b(255, 0, 0));
    toRGB(cloud2, rgb2, cv::Vec3b(0, 255, 0));
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb1, "cloud1");
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb2, "cloud2");
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    // viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread(viewerPsycho);

    pcl::ModelCoefficients line_coeff1;


    viewer->addLine(l1p1,l1p2, 255, 255, 0, "line1");
    viewer->addLine(l2p1,l2p2, 0, 255, 255, "line2");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr c1, pcl::PointCloud<pcl::PointXYZI>::Ptr c2) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb2(new pcl::PointCloud<pcl::PointXYZRGB>);
    toRGB(c1, rgb1, cv::Vec3b(255, 0, 0));
    toRGB(c2, rgb2, cv::Vec3b(0, 255, 0));
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb1, "cloud1");
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb2, "cloud2");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}