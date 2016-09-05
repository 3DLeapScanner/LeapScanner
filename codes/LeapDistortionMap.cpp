
/*

float destinationWidth = 320;
float destinationHeight = 120;
unsigned char destination[(int)destinationWidth][(int)destinationHeight];

//define needed variables outside the inner loop
float calibrationX, calibrationY;
float weightX, weightY;
float dX, dX1, dX2, dX3, dX4;
float dY, dY1, dY2, dY3, dY4;
int x1, x2, y1, y2;
int denormalizedX, denormalizedY;
int i, j;

const unsigned char* raw = image.data();
const float* distortion_buffer = image.distortion();

//Local variables for values needed in loop
const int distortionWidth = image.distortionWidth();
const int width = image.width();
const int height = image.height();

for (i = 0; i < destinationWidth; i += 1) {
	for (j = 0; j < destinationHeight; j += 1) {
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
*/
/*
using namespace cv;
using namespace std;
using namespace Leap;

int main(int argc, char** argv)
{
const float destinationWidth = 320;
const float destinationHeight = 120;
unsigned char destination[(int)destinationWidth][(int)destinationHeight];

//define needed variables outside the inner loop
float calibrationX, calibrationY;
float weightX, weightY;
float dX, dX1, dX2, dX3, dX4;
float dY, dY1, dY2, dY3, dY4;
int x1, x2, y1, y2;
int denormalizedX, denormalizedY;
int i, j;

// Create 640x240 mat for window
cv::Mat win_mat(cv::Size(640, 240), CV_8UC1);

Controller controller;
controller = Controller();
Frame frame = controller.frame();


Mat imgCorrected1(destinationHeight, destinationWidth, CV_8UC1);
Mat imgCorrected2(destinationHeight, destinationWidth, CV_8UC1);


controller.setPolicy(Leap::Controller::POLICY_IMAGES);

ImageList images = controller.images();
for (int img = 0; img < 2; img++) {
Image image = images[img];

const unsigned char* raw = image.data();
const float* distortion_buffer = image.distortion();

//Local variables for values needed in loop
const int distortionWidth = image.distortionWidth();
const int width = image.width();
const int height = image.height();

for (i = 0; i < destinationWidth; i++) {
for (j = 0; j < destinationHeight; j++) {
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
if (img == 0) {
for (int i1 = 0; i1 < destinationHeight; i1++)
for (int j1 = 0; j1 < destinationWidth; j1++)
imgCorrected1.at<unsigned char>(i1, j1) = destination[j1][i1];
}
else {
for (int i1 = 0; i1 < destinationHeight; i1++)
for (int j1 = 0; j1 < destinationWidth; j1++)
imgCorrected2.at<unsigned char>(i1, j1) = destination[j1][i1];
}
// Copy small images into big mat
imgCorrected1.copyTo(win_mat(cv::Rect(0, 0, 320, 120)));
imgCorrected2.copyTo(win_mat(cv::Rect(320, 0, 320, 120)));

// Display big mat
cv::imshow("Images", win_mat);
}
}

*/