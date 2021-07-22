#include "lanedetection.h"
#include "open3d/Open3D.h"
#include "utils.h"
using namespace std;
int main(int argc, char* argv[])
{
	cv::Mat uimage = cv::imread(argv[1], cv::IMREAD_ANYDEPTH);
	cv::imshow("", uimage);
	cv::waitKey(0);
	findLaneInImage(uimage);

	return 0;

}