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

static int ct = 0;
static int counter = 0;

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
	stringstream ss1, ss2;

	string Lname = "Left_";
	string Rname = "Right_";

	string type = ".jpg";

	ss1 << Lname << (ct + 1) << type;
	ss2 << Rname << (ct + 1) << type;

	string filename1 = ss1.str();
	string filename2 = ss2.str();

	ss1.str("");
	ss2.str("");


	Mat leftMat, rightMat;
	if (images.count() == 2)
	{
		leftMat = Mat(images[0].height(), images[0].width(), CV_8UC1, (void *)images[0].data());
		rightMat = Mat(images[1].height(), images[1].width(), CV_8UC1, (void *)images[1].data());
		imshow("leftMat", leftMat);
		imshow("rightMat", rightMat);
		waitKey(1);
		imwrite(filename1, leftMat);
		imwrite(filename2, rightMat);
		cout << counter << endl;
		
	}
	waitKey(1000);

	counter++;
	ct++;

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