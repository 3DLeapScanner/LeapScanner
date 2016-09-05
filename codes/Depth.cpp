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
	Mat img1, img2;
	Mat leftMat, rightMat;
	img1 = imread("tsukuba.png", CV_8UC1);
	//img2 = imread("Right1.jpg", CV_8UC1);
	
	Mat CM1 = Mat(3, 3, CV_32FC1);
	Mat CM2 = Mat(3, 3, CV_32FC1);
	Mat D1, D2, R, T;
	Mat R1 = Mat(3,3,CV_32FC1);  
	Mat R2 = Mat(3, 3, CV_32FC1);
	Mat P1 = Mat(3, 4, CV_32FC1);
	Mat P2 = Mat(3, 4, CV_32FC1);
	Mat Q = Mat(4, 4, CV_32FC1);
	FileStorage fs("test.yml", FileStorage::READ);

	fs["M1"] >> CM1;
	fs["M2"] >> CM2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;
	fs["R"] >> R;
	fs["T"] >> T;
	
	
	
	stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q,CV_CALIB_ZERO_DISPARITY , -1);
	fs.open("test2.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}

	cout << "Recitification done" << endl;

	reprojectImageTo3D(img1, img2, Q, false, -1);
	
	imshow("Depth",img2);
	imwrite("tsukuba_Depth.png", img2);
	waitKey(1);
	cin.get();
	return 0;

}