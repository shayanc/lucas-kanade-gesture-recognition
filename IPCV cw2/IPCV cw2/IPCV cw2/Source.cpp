#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

const int REGION_SIZE = 10;

void calc_derivatives(Mat currFrame, Mat prevFrame, Mat &dx, Mat &dy, Mat &dt);
void LKTracker(Mat &dx, Mat &dy, Mat &dt, Mat &frame);

int main(int argc, const char** argv)
{
	cv::VideoCapture cap;
	Mat currentFrame, previousFrame, dx, dy, dt;

	//VIDEO CAPTURE
	if (argc > 1) cap.open(string(argv[1]));
	else cap.open(CV_CAP_ANY);
	if (!cap.isOpened()) printf("Error: could not load a camera or video.\n");

	cap >> currentFrame;
	cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);
	previousFrame = currentFrame.clone();
	dx.create(currentFrame.rows - 1, currentFrame.cols - 1, CV_64F);
	dy.create(currentFrame.rows - 1, currentFrame.cols - 1, CV_64F);
	dt.create(currentFrame.rows - 1, currentFrame.cols - 1, CV_64F);


	//CREATE DX,DY,DT SO WE CAN CHECK SIZE LATER

	//RUN LOOP
	for (;;) {

		//~60fps
		waitKey(16);	
		//get frame from video capture
		cap >> currentFrame;
		cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);
		//flip horizontally so it does not look inverted
		flip(currentFrame, currentFrame, 1);	
		if (!currentFrame.data) {
			printf("Error: no frame data.\n");
			break;
		}

		//COMPUTE DERIVATIVES
		calc_derivatives(currentFrame, previousFrame, dx, dy, dt);
		//COMPUTE LUCAS K ALGORITHM
		LKTracker(dx, dy, dt, currentFrame);

		previousFrame = currentFrame.clone();
		imshow("video", currentFrame);			// Display content of frameOriginal
	}

	return 0;
}


void calc_derivatives(Mat currFrame, Mat prevFrame, Mat &dx, Mat &dy, Mat &dt)
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
	//normalize(dt, dt, , , NORM_MINMAX);
	imshow("px", dx);
	imshow("py", dy);
	imshow("pt", dt);
	*/
}

void LKTracker(Mat &dx, Mat &dy, Mat &dt, Mat &frame)
{
	int yy_max, xx_max;
	

	for (int y = 0; y < frame.rows; y += REGION_SIZE)
		for (int x = 0; x < frame.cols; x += REGION_SIZE) {
		yy_max = min(y+REGION_SIZE, frame.rows-1);
		xx_max = min(x+REGION_SIZE, frame.cols-1);

		//http://docs.opencv.org/modules/core/doc/basic_structures.html
		//Mat A = Mat::zeros(2, 2, CV_32F);
		//double A[2][2] = { { 0, 0 }, { 0, 0 } };
		//double B[2] = { 0, 0 };
		Matx<double, 2, 2> A(0, 0, 0, 0);
		Vec2d B(0, 0);
		for (int jj = y; jj < yy_max; jj++)
			for (int ii = x; ii < xx_max; ii++){
			
			//compute Ix^2
			A(0, 0) += dx.at<double>(jj, ii) * dx.at<double>(jj, ii);
			//cout << A(0, 0) << "A" << endl;
			//compute Ix*Iy
			A(0, 1) += dx.at<double>(jj, ii) * dy.at<double>(jj, ii);
			//compute Iy^2
			A(1, 1) = dy.at<double>(jj, ii) * dy.at<double>(jj, ii);
			//compute Ix*It
			B[0] += dx.at<double>(jj, ii) * dt.at<double>(jj, ii);
			//compute Iy*It
			B[1] += dy.at<double>(jj, ii) * dt.at<double>(jj, ii);
			}
		A(1, 0) = A(0, 1);
		Vec2d velocities = A.inv()*B;

		//cout << B[1] << endl;
		
		//if (result(0) != 0)
			//cout << result(0) << " " << result(1) << endl;
	
		/* //why is Matx*vec2d much much faster then Mat*Mat?
		Mat AA = Mat(2, 2, CV_64F, A).inv();
		Mat BB = Mat(2, 1, CV_64F, B);
		//Mat V = Mat(2, 2, CV_64F);
		Mat V = AA.inv()*BB;
		int i = V.at<double>(0, 0);
		*/
		//Vec2d c = AA*B;
		}
}