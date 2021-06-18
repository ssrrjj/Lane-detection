/*
 * pcshow.cpp
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */

#include "pcshow.h"


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
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //blocks until the cloud is actually rendered
  viewer.showCloud(cloud);
  //use the following functions to get access to the underlying more advanced/powerful
  //PCLVisualizer

  //This will only get called once
  // viewer.runOnVisualizationThreadOnce (viewerOneOff);

  //This will get called once per visualization iteration
  viewer.runOnVisualizationThread (viewerPsycho);
  while (!viewer.wasStopped ())
  {
  //you can also do cool processing here
  //FIXME: Note that this is running in a separate thread from viewerPsycho
  //and you should guard against race conditions yourself...
  user_data++;
  }
}

