#include "curbdetect.h"
#include "lasStream.h"
#include <filesystem>

using namespace std::filesystem;
int main(int argc, char* argv[]) {
	std::string filename(argv[1]);
	cout << "curb detection " << filename << endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::io::loadPCDFile(filename, *cloud);
	CloudPtr result = FindCurb(cloud);
	
	vector<string> path_name = SplitFilename(filename);
	if (!exists(path_name[0] + "/curb"))
	{
		create_directory(path_name[0] + "/curb");
	}
	string output_file_path = path_name[0] + "/curb/" + path_name[1];
	cout << result->points.size() << endl;
	cout << output_file_path << endl;
	savepcd(result, output_file_path);
	
	
	return 0;

}