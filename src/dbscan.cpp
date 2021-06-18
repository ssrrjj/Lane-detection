#include "dbscan.h"


void
DBSCAN::setInputCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud)
{
  cloud = inputCloud;
  kdtree.setInputCloud (cloud);
}

void
DBSCAN::setInputCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (*inputCloud, *cloudXYZ); // TODO clean cloud
  cloud = cloudXYZ;
  kdtree.setInputCloud (cloud);
}

void
DBSCAN::regionQuery (pcl::PointXYZ &searchPoint, double radius,
		     std::vector<int> &pointIdxRadiusSearch)
{
  pointIdxRadiusSearch.clear ();
  std::vector<float> pointRadiusSquaredDistance;
  kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch,
		       pointRadiusSquaredDistance);
}

void
DBSCAN::expandCluster (int PtIdx, std::vector<int> &neighborPts,
		       std::vector<int> &cluster, float eps, int minPts)
{
  std::vector<bool> visited; // TODO
  cluster.push_back (PtIdx);
  // for each point P' in neighborPts, expand cluster
  for (int j = 0; j < neighborPts.size (); j++)
    {
      // if P' is not visited
      if (!visited[neighborPts[j]])
	{
	  // Mark P' as visited
	  visited[neighborPts[j]] = true;
	}
    }
}

std::vector<int>
DBSCAN::segment (float eps, int minPts)
{
  // vector<int> cluster;
  int noKeys = cloud->points.size ();
  std::vector<bool> visited (noKeys, false);
  std::vector<bool> clustered (noKeys, false);
  std::vector<int> neighborPts;
  std::vector<int> neighborPts_;
  std::vector<int> noisePts;
  std::vector<int> clusteringIdx (noKeys, -1); // default noise points assign cluster id -1

  std::vector<bool> neighborHash (noKeys, false); // indicate if i is already in neighborPts

  int clusterIdx = 0;
  for (int i = 0; i < noKeys; i++)
    {
      if (!visited[i])
	{
	  visited[i] = true;
	  regionQuery (cloud->points[i], eps, neighborPts);
	  if (neighborPts.size () < minPts)
	    {
	      noisePts.push_back (i);
	    }
	  else
	    {
	      // How to label cluster
	      clusters.push_back (std::vector<int> ()); // cluster.clear(); // start a new cluster
	      // expand cluster, add P to cluster
	      // expandCluster(i, neighborPts, cluster, eps, minPts);
	      clusters[clusterIdx].push_back (i);
	      clusteringIdx[i] = clusterIdx;
	      clustered[i] = true;
	      // for each point P' in neighborPts, expand cluster
	      for (int j = 0; j < neighborPts.size (); j++)
		{
		  // if P' is not visited
		  if (!visited[neighborPts[j]])
		    {
		      // Mark P' as visited
		      visited[neighborPts[j]] = true;
		      regionQuery (cloud->points[neighborPts[j]], eps,
				   neighborPts_);
		      if (neighborPts_.size () >= minPts)
			{
			  for(const auto& pt: neighborPts_)
			    if(!neighborHash[pt]) {
				neighborPts.push_back(pt);
				neighborHash[pt]=true;
			    }
//			  neighborPts.insert (neighborPts.end (),
//					      neighborPts_.begin (),
//					      neighborPts_.end ());
			}
		    }
		  // if P' is not yet a member of any cluster, add P' to cluster c
		  if (!clustered[neighborPts[j]])
		    {
		      clustered[neighborPts[j]] = true;
		      clusters[clusterIdx].push_back (neighborPts[j]);
		      clusteringIdx[neighborPts[j]] = clusterIdx;
		      // clusters.at(c).clusterPoints.push_back(octreeGen->getCloud()->points.at(neighborPts[j]));
		    }
		}
	      clusterIdx++;
	    }
	}
    }

#ifdef DEBUG
  std::cout << clusterIdx << " clusters found!" << std::endl;
#endif

  return clusteringIdx;
}

int
regionQueryByKdtree (pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
		     pcl::PointXYZ &searchPoint, double radius,
		     std::vector<int> &pointIdxRadiusSearch)
{
  // Use kdtreto query points within a radius
  // May use kd tree to improve performance
  // Neighbors within radius search
  std::vector<float> pointRadiusSquaredDistance;

  return kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch,
			      pointRadiusSquaredDistance);
}

