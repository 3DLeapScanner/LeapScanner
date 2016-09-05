#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>

using namespace cv;
using namespace std;


int main(){

	Mat output;
	Mat input = imread("disparity_hand_Undistorted.jpg",CV_64F);
	output=input(Rect(10,20,320,200));

	waitKey(100);
	imshow("output",output);
	imwrite("disparity_hand_Undistorted.jpg",output);
	waitKey(1000);
	cin.get();
	return 0;

}