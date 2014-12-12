#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void calc_derivatives(Mat currFrame, Mat prevFrame, Mat &dx, Mat &dy, Mat dt);

int main(int argc, const char** argv)
{
	cv::VideoCapture cap;
	Mat currentFrame, previousFrame, dx, dy, dt;

	//VIDEO CAPTURE
	if (argc > 1)
		cap.open(string(argv[1]));
	else
		cap.open(CV_CAP_ANY);
	if (!cap.isOpened())
		printf("Error: could not load a camera or video.\n");
	cap >> currentFrame;
	cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);

	//CREATE DX,DY,DT SO WE CAN CHECK SIZE LATER
	dx.create(currentFrame.rows - 1, currentFrame.cols - 1, CV_64F);
	dy.create(currentFrame.rows - 1, currentFrame.cols - 1, CV_64F);
	dt.create(currentFrame.rows - 1, currentFrame.cols - 1, CV_64F);

	//RUN LOOP
	for (;;) {

		previousFrame = currentFrame.clone();
		waitKey(16);								//~60fps
		cap >> currentFrame;						//get frame from video capture
		cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);
		flip(currentFrame, currentFrame, 1);		//flip horizontally so it does not look inverted
		if (!currentFrame.data) {
			printf("Error: no frame data.\n");
			break;
		}

		calc_derivatives(currentFrame, previousFrame, dx, dy, dt);


		previousFrame = currentFrame.clone();
		imshow("video", currentFrame);				// Display content of frameOriginal
	}
	return 0;
}


void calc_derivatives(Mat currFrame, Mat prevFrame, Mat &dx, Mat &dy, Mat dt)
{
	//derivatives of current frame
	Mat dx_cf, dy_cf;
	//derivatives of previous frame
	Mat dx_pf, dy_pf;
	//time derivative
	dt = prevFrame - currFrame;

	//CALC DERIVATIES
	
	for (int i = 0; i < currFrame.rows - 1; i++) {
		for (int j = 0; j < currFrame.cols - 1; j++) {
			double a, b, c, d;

			// Get dx gradient by calculating difference between two adjacent points
			// in both currFrames
			a = currFrame.at<uchar>(i, j + 1) - currFrame.at<uchar>(i, j);
			b = currFrame.at<uchar>(i + 1, j + 1) - currFrame.at<uchar>(i + 1, j);
			c = prevFrame.at<uchar>(i, j + 1) - prevFrame.at<uchar>(i, j);
			d = prevFrame.at<uchar>(i + 1, j + 1) - prevFrame.at<uchar>(i + 1, j);
			dx.at<double>(i, j) = (a + b + c + d) / 4;

			// Get dy gradient by calculating difference between two adjacent points
			// in both currFrames
			a = currFrame.at<uchar>(i + 1, j) - currFrame.at<uchar>(i, j);
			b = currFrame.at<uchar>(i + 1, j + 1) - currFrame.at<uchar>(i, j + 1);
			c = prevFrame.at<uchar>(i + 1, j) - prevFrame.at<uchar>(i, j);
			d = prevFrame.at<uchar>(i + 1, j + 1) - prevFrame.at<uchar>(i, j + 1);
			dy.at<double>(i, j) = (a + b + c + d) / 4;
		}
	}
	/*
	normalize(dx, dx, 0, 1, NORM_MINMAX);
	normalize(dy, dy, 0, 1, NORM_MINMAX);
	normalize(dt, dt, 0, 1, NORM_MINMAX);
	imshow("px", dx);
	imshow("py", dy);
	imshow("pt", dt);
	*/
}