#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

void cloud_viewer(){
	 PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPCDFile ("office2.pcd", *cloud);

    visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);

	while (!viewer.wasStopped ()){}
}
int main ()
{
   cloud_viewer();
    return 0;
}
