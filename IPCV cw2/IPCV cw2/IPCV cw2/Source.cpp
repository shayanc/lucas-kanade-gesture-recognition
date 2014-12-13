#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <numeric>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

const int REGION_SIZE = 15;


void calc_derivatives(Mat currFrame, Mat prevFrame, Mat &dx, Mat &dy, Mat &dt)
{
	//derivatives of current frame
	Mat dx_cf, dy_cf;
	//derivatives of previous frame
	Mat dx_pf, dy_pf;

	dx.create(currFrame.rows - 1, currFrame.cols - 1, CV_64F);
	dy.create(currFrame.rows - 1, currFrame.cols - 1, CV_64F);
	dt.create(currFrame.rows - 1, currFrame.cols - 1, CV_64F);


	//time derivative
	//dt = prevFrame - currFrame;



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


			a = prevFrame.at<uchar>(i, j) - currFrame.at<uchar>(i, j);
			b = prevFrame.at<uchar>(i + 1, j) - currFrame.at<uchar>(i + 1, j);
			c = prevFrame.at<uchar>(i, j + 1) - currFrame.at<uchar>(i, j + 1);
			d = prevFrame.at<uchar>(i + 1, j + 1) - currFrame.at<uchar>(i + 1, j + 1);
			dt.at<double>(i, j) = (a + b + c + d) / 4;


		}
	}
	blur(dt, dt, Size(3, 3));

	//copyMakeBorder(dy, dx, 0, 0, 0, 1, BORDER_REPLICATE);
	//copyMakeBorder(dx, dy, 0, 1, 0, 0, BORDER_REPLICATE);

	/*
	normalize(dx, dx, 0, 255, NORM_MINMAX);
	normalize(dy, dy, 0, 255, NORM_MINMAX);
	normalize(dt, dt, 0, 255, NORM_MINMAX);
	imshow("px", dx);
	imshow("py", dy);
	imshow("pt", dt);
	*/
}

void displayLRX(int row, int col, vector<Point2i> positions, vector<Vec2d> velocities)
{
	Mat_<double> x_component(row, col, 0.5);
	Mat_<unsigned char> motionToLeft(row, col, UCHAR_MAX + 1);
	Mat_<unsigned char> motionToRight(row, col, UCHAR_MAX + 1);
	

	cout << velocities.size() << endl;
	//normalize(vels, vels, 0, 1, NORM_MINMAX);
	double sumX = 0;
	double sumY = 0;
	
	for (int i = 0; i < velocities.size(); i++) {
		Vec2d vel = velocities.at(i);
		double Vx = vel(0);
		double Vy = vel(1);
		//sumX += vel(0);
		//sumY += vel(1);

		//velocity is atleast 4 either way, bigger the value more confident we are,
		if (abs(Vx) > 8 && abs(Vx) / abs(Vy) > 1)  {
			if (Vx > 0 ) 
				rectangle(motionToRight, Rect(positions.at(i).x, positions.at(i).y, REGION_SIZE*5, REGION_SIZE*5), Scalar(255), CV_FILLED);
			else
				rectangle(motionToLeft, Rect(positions.at(i).x, positions.at(i).y, REGION_SIZE*5, REGION_SIZE*5), Scalar(255), CV_FILLED);
		}
		
			
	}
	/*
	if (abs(sumX)>4 && abs(sumX)/abs(sumY)>5){
		if (sumX>0)
			rectangle(motionToRight, Rect(0, 0, col, row), Scalar(255), CV_FILLED);
		else
			rectangle(motionToLeft, Rect(0, 0, col, row), Scalar(255), CV_FILLED);

	}
	*/
		
	//blur(motionToLeft, motionToLeft, Size(10,10));


	//blur(motionToRight, motionToRight, Size(10,10));

	imshow("left", motionToLeft);
	imshow("right", motionToRight);
}

void LKTracker(Mat dx, Mat dy, Mat dt, Mat &frame)
{
	int yy_max, xx_max;
	vector<Point2i> positions;
	vector<Vec2d> velocities;
	const int midshift = REGION_SIZE / 2;

	for (int y = 0; y < frame.rows - 1; y += REGION_SIZE) {
		for (int x = 1; x < frame.cols - 1; x += REGION_SIZE) {
			yy_max = min(y + REGION_SIZE, frame.rows - 2);
			xx_max = min(x + REGION_SIZE, frame.cols - 2);

			//http://docs.opencv.org/modules/core/doc/basic_structures.html


			//Mat A = Mat::zeros(2, 2, CV_32F);
			//double A[2][2] = { { 0, 0 }, { 0, 0 } };
			//double B[2] = { 0, 0 };

			//Mat matA;
			//matA.create(2, 2, CV_64F);
			//Mat matB;
			//matB.create(2, 1, CV_64F);

			Matx<double, 2, 2> A(0, 0, 0, 0);
			Vec2d B(0, 0);


			for (int jj = y; jj < yy_max; jj++) {
				for (int ii = x; ii < xx_max; ii++){

					//compute Ix^2
					A(0, 0) += dx.at<double>(jj, ii) * dx.at<double>(jj, ii);
					//cout << A(0, 0) << "A" << endl;
					//compute Ix*Iy
					A(0, 1) += dx.at<double>(jj, ii) * dy.at<double>(jj, ii);
					//compute Iy^2
					A(1, 1) += dy.at<double>(jj, ii) * dy.at<double>(jj, ii);
					//compute Ix*It
					B[0] += dx.at<double>(jj, ii) * dt.at<double>(jj, ii);
					//compute Iy*It
					B[1] += dy.at<double>(jj, ii) * dt.at<double>(jj, ii);
				}
			}

			//WHY IS MAT_X*VEC2D Sooooooooooo much faster then Mat*Mat???????
			Vec2d vel = A.inv()*B;
		
			/* //why is Matx*vec2d much much faster then Mat*Mat?
			Mat AA = Mat(2, 2, CV_64F, A).inv();
			Mat BB = Mat(2, 1, CV_64F, B);
			//Mat V = Mat(2, 2, CV_64F);
			Mat V = AA.inv()*BB;
			int i = V.at<double>(0, 0);
			*/
			//Vec2d c = AA*B;

			double Vx, Vy;
			Vx = vel(0);//matV.at<double>(0, 0);
			Vy = vel(1); //matV.at<double>(1, 0);	
		
			Point centre, vector, start;
			centre.x = x+midshift;
			centre.y = y+midshift;
			vector.x = centre.x + (Vx > REGION_SIZE*3 ? REGION_SIZE*3 : Vx);
			vector.y = centre.y + (Vy > REGION_SIZE*3 ? REGION_SIZE*3 : Vy);
			start.x = x;
			start.y = y;


			double linelenght = sqrt((centre.x - vector.x) ^ 2 + (centre.y - vector.y) ^ 2);

			double magnitude = sqrt(Vx*Vx + Vy*Vy);

			//only add usefull vectors
			if (magnitude > 2) 	{
				line(frame, centre, vector, Scalar(0, 255, 255), 1, CV_AA);
				circle(frame, vector, 1, Scalar(255, 0, 0), 2);
				positions.push_back(start);
				velocities.push_back(vel);
			}

		}
	}

	displayLRX(frame.rows, frame.cols, positions, velocities);
}


int main(int argc, const char** argv)
{
	cv::VideoCapture cap;
	Mat currentFrame, bwFrame, previousFrame, dx, dy, dt;

	//VIDEO CAPTURE
	if (argc > 1) cap.open(string(argv[1]));
	else cap.open(CV_CAP_ANY);
	if (!cap.isOpened()) printf("Error: could not load a camera or video.\n");

	cap >> currentFrame;
	cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);
	currentFrame.copyTo(previousFrame);
	//previousFrame.convertTo(previousFrame, CV_32F);



	//CREATE DX,DY,DT SO WE CAN CHECK SIZE LATER

	//RUN LOOP
	for (;;) {

		//~60fps
		waitKey(16);
		//get frame from video capture
		cap >> currentFrame;
		if (!currentFrame.data) {
			printf("Error: no frame data.\n");
			break;
		}
		//flip horizontally so it does not look inverted
		flip(currentFrame, currentFrame, 1);
		cvtColor(currentFrame, bwFrame, CV_BGR2GRAY);
		//bwFrame.convertTo(bwFrame, CV_32F);



		//COMPUTE DERIVATIVES
		calc_derivatives(bwFrame, previousFrame, dx, dy, dt);
		//COMPUTE LUCAS K ALGORITHM
		LKTracker(dx, dy, dt, currentFrame);

		bwFrame.copyTo(previousFrame);
		imshow("video", currentFrame);			// Display content of frameOriginal
	}

	return 0;
}
