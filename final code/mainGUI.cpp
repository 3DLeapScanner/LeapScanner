#include <opencv2\opencv.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2\ximgproc\disparity_filter.hpp>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <string>
#include "opencv2/features2d/features2d.hpp"
#include "Leap.h"


using namespace Leap;
using namespace std;
using namespace cv;


static int ct = 0;

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


	Mat leftMat, rightMat;


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

	imwrite("Left_1.tiff", leftMat);
	imwrite("Right_1.tiff", rightMat);


}


void SampleListener::onExit(const Controller& controller)
{
	cout << "Exit" << endl;
}

String distortion(String image){

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
	string Imagename = "Image_";

	string type = ".tiff";

	ss << Imagename << (ct + 1) << type;

	string filename = ss.str();
	imshow("undistorted", output);

	imwrite(filename, output);
	ct++;
	return filename;
	waitKey(100);
}
/*
String disparity(String left, String right){
	Mat img1, img2, g1, g2;
	Mat disp, disp8;

	img1 = imread(left, CV_32FC1);
	img2 = imread(right, CV_32FC1);

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
	String filename = "disparity.tif";
	waitKey(500);
	imwrite(filename, disp8);
	waitKey(100);
	return filename;
}
*/
/*
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
	String temp = "hani_Undistorted_Depth.pcd";
	pcl::io::savePCDFileASCII(temp, cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to .pcd file." << std::endl;
	return temp;
}
*/
/*
void cloud_viewer(String viewername){
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	io::loadPCDFile(viewername, *cloud);

	visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	while (!viewer.wasStopped()){}
}
*/
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
	pcl::io::saveVTKFile("triangulation_hani_Undistorted_Depth.vtk", triangles);

	//pcl::io::savePLYFile("triangulation.ply",triangles);

	// Finish


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	pcl::io::loadPolygonFileVTK("triangulation_hani_Undistorted_Depth.vtk", *mesh);
	viewer->addPolygonMesh(*mesh, "view");
	viewer->resetCameraViewpoint("view");
	while (!viewer->wasStopped())
		viewer->spinOnce(100);
}
*/
void Main_func_call(){
	SampleListener listener;
	Controller leap;

	leap.setPolicy(Leap::Controller::POLICY_IMAGES);
	leap.addListener(listener);
	cin.get();
	leap.removeListener(listener);

	// sample image of left and right camera
	String left = "Right_1.tiff";
	String right = "Left_1.tiff";

	// left camera image pass to distortion function
	//String image1 = distortion(left);

	// right camera image pass to distortion function
	//String image2 = distortion(right);

	// disparity map of left & right camera image
	//String disparityfile = disparity(left, right);

	//backcut algorithm

	//custom point cloud writer "not working still in process" d
	//String pcdfile = pcd_writer(bkcut//disparityfile);

	// Simple point cloud visualization
	//cloud_viewer(pcdfile);
	//waitKey(500);

	//destroyAllWindows();


	//3D surface construction on triangulated point clouds
	//Triangulation_viwer(pcdfile);

}



Mat3b canvas;
string buttonText("Tap Here To Start!");
string winName = "3D Handheld Scanner";

Rect button;


void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		//namedWindow("main", WINDOW_AUTOSIZE);
		//imshow("main", canvas);
		if (button.contains(Point(x, y)))
		{

			cout << "Clicked!" << endl;
			Main_func_call();
			rectangle(canvas(button), button, Scalar(0, 0, 255), 2);
		}
	}
	/*if (event == EVENT_LBUTTONUP)
	{
	rectangle(canvas, button, Scalar(200, 200, 200), 2);
	}*/

	imshow(winName, canvas);
	waitKey(1);
}


void callbackfunc(int, void *){


}
int main()
{
	// An image
	Mat img = imread("Back.jpg", 1);

	// Your button
	button = Rect(0, 0, img.cols, 50);
	//button = Rect(50, 20, 30, 30);
	// The canvas
	canvas = Mat3b(img.rows + button.height, img.cols, Vec3b(0, 0, 0));

	// Draw the button
	canvas(button) = Vec3b(255, 255, 255);
	putText(canvas(button), buttonText, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));

	// Draw the image
	img.copyTo(canvas(Rect(0, button.height, img.cols, img.rows)));

	// Setup callback function
	namedWindow(winName);
	setMouseCallback(winName, callBackFunc);


	imshow(winName, canvas);
	waitKey();

	return 0;
}