#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv\cv.h>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;


int main(int argc, const char** argv)
{
	cv::VideoCapture cap;
	if (argc > 1)
		cap.open(string(argv[1]));
	else
		cap.open(CV_CAP_ANY);
	if (!cap.isOpened())
		printf("Error: could not load a camera or video.\n");

	Mat currentFrame, previousFrame;
	cap >> currentFrame;

	for (;;) {
		
		previousFrame = currentFrame.clone();
		waitKey(16);									//~60fps
		cap >> currentFrame;						//get data from video capture
		if (!currentFrame.data) {
			printf("Error: no frame data.\n");
			break;
		}

		

		previousFrame = currentFrame.clone();
		imshow("video", currentFrame);				// Display content of frameOriginal
	}
	return 0;
}