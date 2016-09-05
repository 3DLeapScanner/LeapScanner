#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>

using namespace std;
using namespace pcl;
using namespace cv;

void pcd_writer(){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	Mat depth_image = cv::imread("Righthh.jpg", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	depth_image.convertTo(depth_image, CV_32F); // convert the image data to float type 

	if (!depth_image.data)
	{
		std::cerr << "No depth data!!!" << std::endl;
		exit(EXIT_FAILURE);
	}

	const float dc1 = 0.66999066565804488;//0.0030711016;
	const float dc2 = 0.70185345602029203;
	const float fx_d = 1.8139592173744651e+002; //fx
	const float fy_d = 1.8139592173744651e+002; //fy
	const float px_d = 3.1738099136734206e+002; //cx
	const float py_d = 1.3835989671763309e+002; //cy

	cloud.width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing 
	cloud.height = depth_image.rows;

	cloud.resize(cloud.width*cloud.height);
	for (int v = 0; v < depth_image.rows; v++)
		//2-D indexing 
		for (int u = 0; u < depth_image.cols; u++)
		{
			float z = 1.0f / (depth_image.at<float>(v, u)*dc1 + dc2);
			cloud(u, v).x = z / (u - px_d) * fx_d;
			cloud(u, v).y = z / (v - py_d) * fy_d;
			cloud(u, v).z = z;
		}
	pcl::io::savePCDFileASCII("Righthh2.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
	//cout << "Point cloud created" << endl;
}
void cloud_viewer(){
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	io::loadPCDFile("Righthh2.pcd", *cloud);

	visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	while (!viewer.wasStopped()){}
}

int main()
{
	pcd_writer();
	cloud_viewer();
	cin.get();
}