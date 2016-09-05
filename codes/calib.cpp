#include <iostream>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

int main()
{
	Mat img = imread("left_167.JPG", CV_LOAD_IMAGE_UNCHANGED); 
	if (img.empty()) 
	{
		cout << "Error : Image cannot be loaded..!!" << endl;
		return -1;
	}

	namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); 
	imshow("MyWindow", img); 


	int numBoards = 1;
	int numCornersHor = 4;
	int numCornersVer = 4;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);
	
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;

	Mat gray_image;
	
	vector<Point3f> obj;
	for (int j = 0; j<numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	bool found = findChessboardCorners(img, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

	if (found)
	{
		cornerSubPix(img, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(img, board_sz, corners, found);
	}
	imshow("win1", img);
		//imshow("win2", gray_image);
	waitKey(0); 
	destroyWindow("MyWindow"); 
	return 0;
}