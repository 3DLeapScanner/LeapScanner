#include <vector>
#include <cstdlib>
#include <iostream>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "Leap.h"

using namespace cv;
using namespace std;
using namespace Leap;

	
	
int main()
{
	Mat img1, img2;
	Mat leftMat, rightMat;
	Mat M1 = Mat(3, 3, CV_64FC1);
	Mat M2 = Mat(3, 3, CV_64FC1);
	Mat D1, D2;
	leftMat = imread("LeftHani.jpg", CV_8UC1);

	imshow("left", leftMat);

	FileStorage fs1("intrinsics.yml", FileStorage::READ);

	fs1["M1"] >> M1;
	fs1["M2"] >> M2;
	fs1["D1"] >> D1;
	fs1["D2"] >> D2;

	//imshow("left", leftMat);
	//undistort(rightMat, img2, CM2, D2);
	waitKey(1000);
	undistort(leftMat, img1, M1, D1);
	//waitKey(1000);

	imshow("Left", img1);
	waitKey(1000);

	//imshow("right", img2);
	cin.get();
	return 0;

}