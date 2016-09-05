#include <iostream>
#include "Leap.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

using namespace std;
using namespace Leap;
using namespace cv;

int main()
{
	String filename = "Right9.jpg";
	Mat leftMat = imread(filename,CV_32FC1);

	imshow("initial", leftMat);
	waitKey(20);

	const unsigned int cmWidth = 256;
	const unsigned int cmHeight = 256;

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
	resize(cMapMatX, cMapMatX, leftMat.size(), 0, 0, CV_INTER_LINEAR);
	resize(cMapMatY, cMapMatY, leftMat.size(), 0, 0, CV_INTER_LINEAR);


	// now the calibration map has the size of your original image, but its values are still between 0 and 1 (for legal positions)
	// so scale to image size:
	cMapMatX = leftMat.cols * cMapMatX;
	cMapMatY = leftMat.rows * cMapMatY;


	// now create undistorted image:
	Mat undistortedImage = Mat(leftMat.rows, leftMat.cols, CV_8UC3);
	undistortedImage.setTo(Vec3b(0, 0, 0));   // initialize black

	
	for (int j = 0; j<undistortedImage.rows; ++j)
		for (int i = 0; i<undistortedImage.cols; ++i)
		{
			Point undistPosition;
			undistPosition.x = (cMapMatX.at<float>(j, i)); // this will round the position, maybe you want interpolation instead
			undistPosition.y = (cMapMatY.at<float>(j, i));

			if (undistPosition.x >= 0 && undistPosition.x < leftMat.cols
				&& undistPosition.y >= 0 && undistPosition.y < leftMat.rows)

			{
				undistortedImage.at<Vec3b>(j, i) = leftMat.at<Vec3b>(undistPosition);
			}

		}
	
	imshow("undistorted", undistortedImage);

	imwrite("dst9.jpg", undistortedImage);
	waitKey(0);
}


Mat SelfDescriptorDistances(Mat descr)
{
	Mat selfDistances = Mat::zeros(descr.rows, descr.rows, CV_64FC1);
	for (int keyptNr = 0; keyptNr < descr.rows; ++keyptNr)
	{
		for (int keyptNr2 = 0; keyptNr2 < descr.rows; ++keyptNr2)
		{
			double euclideanDistance = 0;
			for (int descrDim = 0; descrDim < descr.cols; ++descrDim)
			{
				double tmp = descr.at<float>(keyptNr, descrDim) - descr.at<float>(keyptNr2, descrDim);
				euclideanDistance += tmp*tmp;
			}

			euclideanDistance = sqrt(euclideanDistance);
			selfDistances.at<double>(keyptNr, keyptNr2) = euclideanDistance;
		}

	}
	return selfDistances;
}