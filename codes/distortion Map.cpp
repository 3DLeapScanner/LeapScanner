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
	const int destinationWidth = 320;
	const int destinationHeight = 120;
	unsigned char destination[(int)destinationWidth][(int)destinationHeight];

	//define needed variables outside the inner loop
	float calibrationX, calibrationY;
	float weightX, weightY;
	float dX, dX1, dX2, dX3, dX4;
	float dY, dY1, dY2, dY3, dY4;
	int x1, x2, y1, y2;
	int denormalizedX, denormalizedY;
	int i, j;

	for (int i = 0; i < 2; i++)
	{
		Image image = images[i];

		const unsigned char* raw = image.data();
		const float* distortion_buffer = image.distortion();

		//Local variables for values needed in loop
		const int distortionWidth = image.distortionWidth();
		const int width = image.width();
		const int height = image.height();

		for (i = 0; i < destinationWidth; i += 1)
		{
			for (j = 0; j < destinationHeight; j += 1)
			{
				//Calculate the position in the calibration map (still with a fractional part)
				calibrationX = 63 * i / destinationWidth;
				calibrationY = 62 * (1 - j / destinationHeight); // The y origin is at the bottom
				//Save the fractional part to use as the weight for interpolation
				weightX = calibrationX - truncf(calibrationX);
				weightY = calibrationY - truncf(calibrationY);

				//Get the x,y coordinates of the closest calibration map points to the target pixel
				x1 = calibrationX; //Note truncation to int
				y1 = calibrationY;
				x2 = x1 + 1;
				y2 = y1 + 1;

				//Look up the x and y values for the 4 calibration map points around the target
				dX1 = distortion_buffer[x1 * 2 + y1 * distortionWidth];
				dX2 = distortion_buffer[x2 * 2 + y1 * distortionWidth];
				dX3 = distortion_buffer[x1 * 2 + y2 * distortionWidth];
				dX4 = distortion_buffer[x2 * 2 + y2 * distortionWidth];
				dY1 = distortion_buffer[x1 * 2 + y1 * distortionWidth + 1];
				dY2 = distortion_buffer[x2 * 2 + y1 * distortionWidth + 1];
				dY3 = distortion_buffer[x1 * 2 + y2 * distortionWidth + 1];
				dY4 = distortion_buffer[x2 * 2 + y2 * distortionWidth + 1];

				std::cout << i << ", " << j << " -- " << x1 << ", " << y1 << ", " << x2 << ", " << y2 << " -- "
					<< (x1 * 2 + y1 * distortionWidth) << ", "
					<< (x1 * 2 + y1 * distortionWidth + 1) << " -- "
					<< (x2 * 2 + y2 * distortionWidth) << ", "
					<< (x2 * 2 + y2 * distortionWidth + 1) << std::endl;
				//Bilinear interpolation of the looked-up values:
				// X value
				dX = dX1 * (1 - weightX) * (1 - weightY) +
					dX2 * weightX * (1 - weightY) +
					dX3 * (1 - weightX) * weightY +
					dX4 * weightX * weightY;

				// Y value
				dY = dY1 * (1 - weightX) * (1 - weightY) +
					dY2 * weightX * (1 - weightY) +
					dY3 * (1 - weightX) * weightY +
					dY4 * weightX * weightY;

				// Reject points outside the range [0..1]
				if ((dX >= 0) && (dX <= 1) && (dY >= 0) && (dY <= 1)) {
					//Denormalize from [0..1] to [0..width] or [0..height]
					denormalizedX = dX * width;
					denormalizedY = dY * height;

					//look up the brightness value for the target pixel
					destination[i][j] = raw[denormalizedX + denormalizedY * width];
				}
				else {
					destination[i][j] = -1;
				}
			}
		}

	}
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