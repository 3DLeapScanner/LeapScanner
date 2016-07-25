#include <vector>
#include <cstdlib>
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
/*	int numBoards = 5;
	int numCornersHor = 9;
	int numCornersVer = 6;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);
*/	VideoCapture capture = VideoCapture(0);

/*	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;
	*/
	Mat image;
	Mat gray_image;
	capture >> image;
	/*
	vector<Point3f> obj;
	for (int j = 0; j<numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	while (successes<numBoards)
	{
		cvtColor(image, gray_image, CV_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK);

		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}
		*/
		imshow("win1", image);
		//imshow("win2", gray_image);

		/*capture >> image;

		int key = waitKey(1);

		if (key == 27)
			return 0;

		if (found)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);
			cout << "Snap stored" << endl;

			successes++;

			if (successes >= numBoards)
				break;
		}
	}
	destroyAllWindows();
	cout << "Calibration Starts" << endl;
	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.at<float>(0, 0) = 1;
	intrinsic.at<float>(1, 1) = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	
	FileStorage fs1("hpcalib.yml", FileStorage::WRITE);
	fs1 << "CM1" << intrinsic;
	fs1 << "D1" << distCoeffs;
	*/
	cout << "calibration done" << endl;
/*	Mat imageUndistorted;
	while (1)
	{
		capture >> image;
		undistort(image, imageUndistorted, intrinsic, distCoeffs);

		imshow("win1", image);
		imshow("dis", imageUndistorted);

		waitKey(1);
	}
	*/
	capture.release();

	return 0;
}