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

class SampleListener : public Listener
{
public:
	virtual void onInit(const Controller&);
	virtual void onConnect(const Controller&);
	virtual void onDisconnect(const Controller&);
	virtual void onExit(const Controller&);
	virtual void onFrame(const Controller&);

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
void SampleListener::onFrame(const Controller& controller)
{
	const Frame frame = controller.frame();
	ImageList images = frame.images();
	//int i = 0;
	Mat leftMat;
	Mat rightMat;
	int numBoards = 20;
	int board_w = 9;
	int board_h = 6;

	Size board_sz = Size(board_w, board_h);
	int board_n = board_w*board_h;

	vector<vector<Point3f> > object_points;
	vector<vector<Point2f> > imagePoints1, imagePoints2;
	vector<Point2f> corners1, corners2;

	vector<Point3f> obj;
	for (int j = 0; j<board_n; j++)
	{
		obj.push_back(Point3f(j / board_w, j%board_w, 0.0f));
	}
	int success = 0, k = 0;
	bool found1 = false, found2 = false;
	leftMat = Mat(images[0].height(), images[0].width(), CV_8UC1, (void *)images[0].data());
	rightMat = Mat(images[1].height(), images[1].width(), CV_8UC1, (void *)images[1].data());
	//while (success < numBoards)
	//{
		if (images.count() == 2)
		{
			//leftMat = Mat(images[0].height(), images[0].width(), CV_8UC1, (void *)images[0].data());
			//rightMat = Mat(images[1].height(), images[1].width(), CV_8UC1, (void *)images[1].data());
			found1 = findChessboardCorners(leftMat, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			found2 = findChessboardCorners(rightMat, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

			if (found1)
			{
				cornerSubPix(leftMat, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				drawChessboardCorners(leftMat, board_sz, corners1, found1);
			}

			if (found2)
			{
				cornerSubPix(rightMat, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				drawChessboardCorners(rightMat, board_sz, corners2, found2);
			}
			imshow("leftMat", leftMat);
			imshow("rightMat", rightMat);
			if (found1 != 0 && found2 != 0)
			{
				imagePoints1.push_back(corners1);
				imagePoints2.push_back(corners2);
				object_points.push_back(obj);
				cout <<"Corners stored\n";
				success++;

				if (success >= numBoards)
				{
					return;
				}
			}
			waitKey(1);
		}
	
	cout << "Starting Calibration\n";
	Mat CM1 = Mat(3, 3, CV_64FC1);
	Mat CM2 = Mat(3, 3, CV_64FC1);
	Mat D1, D2;
	Mat R, T, E, F;

	stereoCalibrate(object_points, imagePoints1, imagePoints2,
		CM1, D1, CM2, D2,leftMat.size(), R, T, E, F,
		cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

	FileStorage fs1("Leapcalib.yml", FileStorage::WRITE);
	fs1 << "CM1" << CM1;
	fs1 << "CM2" << CM2;
	fs1 << "D1" << D1;
	fs1 << "D2" << D2;
	fs1 << "R" << R;
	fs1 << "T" << T;
	fs1 << "E" << E;
	fs1 << "F" << F;

	printf("Done Calibration\n");
}

void SampleListener::onExit(const Controller& controller)
{
	cout << "Exit" << endl;
}

int main()
{
	SampleListener listener;
	Controller leap;
	
	
	leap.addListener(listener);
	leap.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);
	leap.setPolicy(Leap::Controller::POLICY_IMAGES);

	std::cin.get();
	leap.removeListener(listener);

	
	return 0;

}