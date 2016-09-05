#include <vector>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
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
	
		imshow("leftMat", leftMat);
		imshow("rightMat", rightMat);
		waitKey(10);

		imwrite("Left_48.jpg", leftMat);
		imwrite("Right_48.jpg", rightMat);


}


void SampleListener::onExit(const Controller& controller)
{
	cout << "Exit" << endl;
}

int main()
{
	SampleListener listener;
	Controller leap;

	leap.setPolicy(Leap::Controller::POLICY_IMAGES);
	leap.addListener(listener);
	cin.get();
	leap.removeListener(listener);


	return 0;

}