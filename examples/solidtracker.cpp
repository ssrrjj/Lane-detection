#include "polyline.h"
#include "solidtrack.h"
#include "LasStream.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <mutex>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
std::mutex cloud_mutex;

struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	CloudPtr cloud;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	static int lineidx = 0;
	static int do_clear = 0;

	struct callback_args* data = (struct callback_args*)args;

	if (event.getPointIndex() == -1)
		return;
	if (do_clear) {
		data->viewerPtr->removeAllShapes();
		do_clear = 0;
		lineidx = 0;
		return;
	}
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	//data->viewerPtr->removePointCloud("clicked_points");
	//data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	//data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	//data->viewerPtr->addSphere(current_point, 0.3, "sphere" + to_string(sphereidx));

	//data->viewerPtr->spin();

	if (data->clicked_points_3d->points.size() == 2) {
		pcl::PointXYZ p1(data->clicked_points_3d->points[0].x, data->clicked_points_3d->points[0].y, data->clicked_points_3d->points[0].z);
		pcl::PointXYZ p2(data->clicked_points_3d->points[1].x, data->clicked_points_3d->points[1].y, data->clicked_points_3d->points[1].z);


		PolyLine line = solidtrack(data->cloud, data->kdtree, p1, p2);
		cout << line.points.size() << endl;
		for (int point_idx = 0; point_idx < line.points.size() - 1; point_idx++) {
			data->viewerPtr->addLine(line.points[point_idx], line.points[point_idx + 1], 255, 255, 255, "line" + to_string(lineidx));
			lineidx++;
		}
		data->clicked_points_3d->points.clear();
		do_clear = 1;
	}


}
void main(int argc, char* argv[])
{
	std::string filename(argv[1]);
	//visualizer
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));

	if (filename[filename.length() - 1] == 'd') {
		if (pcl::io::loadPCDFile(filename, *cloud))
		{
			std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
			return;
		}
	}
	// pcl::io::loadPCDFile ("point_cloud_00007.pcd", *cloud);
	else
		readlas(filename, cloud);
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
	kdtree->setInputCloud(cloud);
	cout << "kdtree" << endl;
	std::cout << cloud->points.size() << std::endl;

	cloud_mutex.lock();    // for not overwriting the point cloud

	// Display pointcloud:
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(cloud, point_cloud_color_handler, "cloud");
	//viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.cloud = cloud;
	cb_args.kdtree = kdtree;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on points, then press 'Q'..." << std::endl;
	// Spin until 'Q' is pressed:
	viewer->spin();
	std::cout << "done." << std::endl;

	cloud_mutex.unlock();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}