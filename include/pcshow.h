/*
 * pcshow.h
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */

#ifndef INCLUDE_PCSHOW_H_
#define INCLUDE_PCSHOW_H_

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

#endif /* INCLUDE_PCSHOW_H_ */
