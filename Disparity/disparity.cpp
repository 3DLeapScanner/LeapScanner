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


int main()
{
	Mat img1, img2, g1, g2;
	Mat disp, disp8;

	img1 = imread("dis9.jpg", CV_8UC1);
	img2 = imread("dst9.jpg", CV_8UC1);
	/*
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
			sbm(img1,img2, disp);
		*/
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
		sgbm(img1,img2,disp);
		
		normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8UC1);
		imshow("left", img1);
		imshow("right", img2);
		imshow("disp", disp8);
		imwrite("ddst.jpg", disp8);

		waitKey(100);
		cin.get();
	return(0);
}