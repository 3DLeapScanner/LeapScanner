/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010, Willow Garage, Inc.
*  Copyright (c) 2012-, Open Perception, Inc.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder(s) nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <opencv2\opencv.hpp>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl\io\vtk_lib_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

#include <cstdlib>
#include <sstream>
#include <string>
#include "opencv2/features2d/features2d.hpp"
#include "Leap.h"


using namespace Leap;
using namespace std;
using namespace pcl;
using namespace cv;
using namespace cv::ximgproc;

static int ct = 0;
static int counter = 0;

class SampleListener : public Listener
{
public:
	virtual void onInit(const Controller&);
	virtual void onConnect(const Controller&);
	virtual void onDisconnect(const Controller&);
	virtual void onExit(const Controller&);
	virtual void onImages(const Controller&);

};

void SampleListener::onInit(const Controller& controller)
{
	cout << "Initialized" << endl;
}
void SampleListener::onConnect(const Controller& controller)
{
	cout << "Connected" << endl;
}
void SampleListener::onDisconnect(const Controller& controller)
{
	cout << "Disconnected" << endl;
}
void SampleListener::onImages(const Controller& controller)
{

	ImageList images = controller.images();
	stringstream ss1, ss2;

	//string Lname = "Left";
	//string Rname = "Right";


	string type = ".jpg";

	ss1 << (ct + 1) << type;
	ss2 << (ct + 1) << type;

	string filename1 = ss1.str();
	string filename2 = ss2.str();

	ss1.str("");
	ss2.str("");

	Mat leftMat, rightMat;
	int count = 0;
	if (images.count() == 2)
	{
		leftMat = Mat(images[0].height(), images[0].width(), CV_8UC1, (void *)images[0].data());
		rightMat = Mat(images[1].height(), images[1].width(), CV_8UC1, (void *)images[1].data());

		//namedWindow("Left&Rigth Stream", WINDOW_AUTOSIZE);
		Mat big(Size(1280, 240), CV_8UC1);

		waitKey(10);
		leftMat.copyTo(big(cv::Rect(0, 0, 640, 240)));
		rightMat.copyTo(big(cv::Rect(640, 0, 640, 240)));

		// Display big mat
		imshow("Video Streaming", big);
		//imshow("leftMat", leftMat);
		//imshow("rightMat", rightMat);
		waitKey(10);
		
		imwrite("streamingData\\" + filename1, leftMat);
		imwrite("streamingData\\" + filename2, rightMat);

	}
	counter++;
	ct++;

}


void SampleListener::onExit(const Controller& controller)
{
	cout << "Exit" << endl;
}

void distortion(String image){

	Mat ImageMat = imread(image, CV_32FC1);

	imshow("initial", ImageMat);
	waitKey(20);

	const unsigned int cmWidth = 320;
	const unsigned int cmHeight = 120;

	// compute the calibration map by transforming image locations to values between 0 and 1 for legal positions.
	float calibMap[cmWidth*cmHeight * 2];
	for (unsigned int y = 0; y < cmHeight; ++y)
		for (unsigned int x = 0; x < cmWidth; ++x)
		{
			float xx = (float)x / (float)cmWidth;
			xx = xx*2.0f;
			float yy = (float)y / (float)cmHeight;

			calibMap[y*cmWidth * 2 + 2 * x] = xx;
			calibMap[y*cmWidth * 2 + 2 * x + 1] = yy;
		}


	// NOW you have the initial situation of your scenario: calibration map and distorted image...

	// compute the image locations of calibration map values:
	Mat cMapMatX = Mat(cmHeight, cmWidth, CV_32FC1);
	Mat cMapMatY = Mat(cmHeight, cmWidth, CV_32FC1);
	for (int j = 0; j<cmHeight; ++j)
		for (int i = 0; i<cmWidth; ++i)
		{
			cMapMatX.at<float>(j, i) = calibMap[j*cmWidth * 2 + 2 * i];
			cMapMatY.at<float>(j, i) = calibMap[j*cmWidth * 2 + 2 * i + 1];
		}


	// interpolate those values for each of your original images pixel:
	// here I use linear interpolation, you could use cubic or other interpolation too.
	resize(cMapMatX, cMapMatX, ImageMat.size(), 0, 0, CV_INTER_LINEAR);
	resize(cMapMatY, cMapMatY, ImageMat.size(), 0, 0, CV_INTER_LINEAR);


	// now the calibration map has the size of your original image, but its values are still between 0 and 1 (for legal positions)
	// so scale to image size:
	cMapMatX = ImageMat.cols * cMapMatX;
	cMapMatY = ImageMat.rows * cMapMatY;


	// now create undistorted image:
	Mat undistortedImage = Mat(ImageMat.rows, ImageMat.cols, CV_8UC3);
	undistortedImage.setTo(Vec3b(0, 0, 0));   // initialize black


	for (int j = 0; j<undistortedImage.rows; ++j)
		for (int i = 0; i<undistortedImage.cols; ++i)
		{
			Point undistPosition;
			undistPosition.x = (cMapMatX.at<float>(j, i)); // this will round the position, maybe you want interpolation instead
			undistPosition.y = (cMapMatY.at<float>(j, i));

			if (undistPosition.x >= 0 && undistPosition.x < ImageMat.cols
				&& undistPosition.y >= 0 && undistPosition.y < ImageMat.rows)

			{
				undistortedImage.at<Vec3b>(j, i) = ImageMat.at<Vec3b>(undistPosition);
			}

		}

	Mat output = undistortedImage(Rect(10, 20, 300, 200));
	waitKey(100);

	stringstream ss;
	//string Imagename = "Image_";

	string type = ".jpg";

	ss << (ct + 1) << type;

	string filename = ss.str();
	imshow("undistorted", output);

	imwrite("streamingData\\distortData\\" + filename, output);
	ct++;
	waitKey(100);
}
void disparity(String leftImage, String rightImage)
{
	//int SADWindowSize = 0;
	
	//String dst_conf_path = parser.get<String>("dst_conf_path");
	String algo = "sgbm";
	String filter = "wls_conf";

	//bool no_downscale = true;
	//  int max_disp = parser.get<int>("max_disparity");
	double lambda = 8000.0;
	double sigma = 2.0;
	double vis_mult = 9.0;
	int sgbmWinSize = 9;
	//int wsize = sgbmWinSize;
	//int numberOfDisparities = 16*3;
	Mat disp, disp8;

	Mat left = imread(leftImage, CV_8U);
	Mat right = imread(rightImage, CV_8U);

	Mat left_for_matcher, right_for_matcher;
	Mat left_disp, right_disp;
	Mat filtered_disp;
	Mat conf_map = Mat(left.rows, left.cols, CV_8U);
	conf_map = Scalar(255);
	Rect ROI;

	Ptr<DisparityWLSFilter> wls_filter;

	left_for_matcher = left.clone();
	right_for_matcher = right.clone();

	resize(left, left_for_matcher, Size(), 0.5, 0.5);
	resize(right, right_for_matcher, Size(), 0.5, 0.5);


	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, 16 , 3);
	left_matcher->setPreFilterCap(63);
	//int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;

	//sgbm->setBlockSize(sgbmWinSize);

	//int cn = left.channels();

	left_matcher->setP1(24 * sgbmWinSize*sgbmWinSize);
	left_matcher->setP2(96 * sgbmWinSize*sgbmWinSize);
	/* sgbm->setMinDisparity(1);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(5);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);*/
	left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
	wls_filter = createDisparityWLSFilter(left_matcher);


	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

	left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
	right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);

	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	wls_filter->filter(left_disp, left, filtered_disp, right_disp);
	conf_map = wls_filter->getConfidenceMap();

	ROI = wls_filter->getROI();

	resize(left_disp, left_disp, Size(), 2.0, 2.0);
	left_disp = left_disp*2.0;
	ROI = Rect(ROI.x * 2, ROI.y * 2, ROI.width * 2, ROI.height * 2);

	Mat raw_disp_vis;
	getDisparityVis(left_disp, raw_disp_vis, vis_mult);
	namedWindow("raw disparity", WINDOW_AUTOSIZE);
	imshow("raw disparity", raw_disp_vis);
	Mat filtered_disp_vis;
	getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
	namedWindow("filtered disparity", WINDOW_AUTOSIZE);
	imshow("filtered disparity", filtered_disp_vis);
	imwrite("streamingData\\filtered_disparity3.jpg", filtered_disp_vis);
	waitKey(0);

	//disp.convertTo(disp8, CV_8U);
	/*  disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

	imshow("WindowDisparity", disp8);

	waitKey(0);*/
	//return filtered_disp_vis;
}

/*
String disparity(String left, String right){
	Mat img1, img2, g1, g2;
	Mat disp, disp8;

	img1 = imread(left, CV_8UC1);
	img2 = imread(right, CV_8UC1);

	StereoBM sbm;
	sbm.state->SADWindowSize = 11;
	sbm.state->numberOfDisparities = 96;
	sbm.state->preFilterSize = 7;
	sbm.state->preFilterCap = 61;
	sbm.state->minDisparity = -39;
	sbm.state->textureThreshold = 1500;
	sbm.state->uniquenessRatio = 0;
	sbm.state->speckleWindowSize = 0;
	sbm.state->speckleRange = 8;
	sbm.state->disp12MaxDiff = 1;
	sbm(img1, img2, disp);
	
	StereoSGBM sgbm;
	sgbm.SADWindowSize = 21;
	sgbm.numberOfDisparities = 128;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = -64;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;
	sgbm(img1, img2, disp);
	
	waitKey(500);
	normalize(disp, disp8, 0, 255, CV_MINMAX, CV_32FC1);
	waitKey(100);
	imshow("left", img1);
	imshow("right", img2);
	waitKey(100);
	imshow("disp", disp8);
	waitKey(100);
	String filename = "disparity.tiff";
	waitKey(500);
	imwrite(filename, disp8);
	waitKey(100);
	return filename;
}
*/

String pcd_writer(String pcdname){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	Mat depth_image = cv::imread(pcdname, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
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
			cloud(u, v).x = z * (u - px_d) / fx_d;
			cloud(u, v).y = z * (v - py_d) / fy_d;
			cloud(u, v).z = z;
		}
	String temp = "pclcode.pcd";
	pcl::io::savePCDFileASCII(temp, cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to .pcd file." << std::endl;
	return temp;
}


void cloud_viewer(String viewername){
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	io::loadPCDFile(viewername, *cloud);

	visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	while (!viewer.wasStopped()){}
}
void openpcd(string name)
{
	Mat depth_image = cv::imread(name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	depth_image.convertTo(depth_image, CV_32F);
	const double max_z = 1.0e4;
	FILE* fp = fopen("openpcd.pcd", "wt");
	for (int y = 0; y < depth_image.rows; y++)
	{
		for (int x = 0; x <depth_image.cols; x++)
		{
			Vec3f point = depth_image.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) 
				continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
	
}
/*
void Triangulation_viwer(String pcdname){

	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(pcdname, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(2.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();


	// Save visualization cloud
	pcl::io::saveVTKFile("custom1.vtk", triangles);

	//pcl::io::savePLYFile("triangulation.ply",triangles);

	// Finish


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	pcl::io::loadPolygonFileVTK("custom1.vtk", *mesh);
	viewer->addPolygonMesh(*mesh, "view");
	viewer->resetCameraViewpoint("view");
	while (!viewer->wasStopped())
		viewer->spinOnce(100);
}
*/
/*
void readfile(vector<string> &filenames, string folder)
{
	path dir(folder);
	directory_iterator itr(dir), end_itr;

	string current_file = itr->path().string();

	for (; itr != end_itr; ++itr)
	{
		if (is_regular_file(itr->path()))
		{
			string filename = itr->path().filename().string(); // returns just filename
			filenames.push_back(filename);
		}
	}
}
*/
int main()
{
	/*String folder = "streamingData";
	vector<string> filenames;
	create_directory("streamingData");

	SampleListener listener;
	Controller leap;

	leap.setPolicy(Leap::Controller::POLICY_IMAGES);
	leap.addListener(listener);
	cin.get();
	leap.removeListener(listener);
	*/
	//sample image of left and right camera
	//String left = "Left_1.jpg";
	//String right ="Right_1.jpg";
	
	//left camera image pass to distortion function
	//String image1 = distortion(left);

	//right camera image pass to distortion function
	//String image2 = distortion(right);

	// disparity map of left & right camera image
	//Mat disparityfile = disparity(image1, image2);
	//disparity(image1, image2);
	//imshow("disparityImage", disparityfile);
	//imwrite("disparity.jpg",disparityfile);
	//backcut algorithm

	//custom point cloud writer "not working still in process" d
	//Mat file = imread("pencil45.png",CV_32F);
	//String pcdfile = pcd_writer("pencil45.png"/*disparityfile*/);
	//String file = "pclcode_pcd.pcd";
	// Simple point cloud visualization
	//openpcd("pencil45.png");
	//string pcdfile = "openpcd.pcd";
	//cloud_viewer(pcdfile);
	waitKey(0);

	//destroyAllWindows();
	

	//3D surface construction on triangulated point clouds
	//Triangulation_viwer("custom1.pcd");

	return 0;
}
