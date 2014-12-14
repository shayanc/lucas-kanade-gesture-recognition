#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <numeric>

#include <iostream>
#include <stdio.h>
#include <errno.h>

using namespace std;
using namespace cv;

int REGION_SIZE = 15;
int MAGNITUDE_FILTER = 5;		//vectors > mag_filter are used to process left and right


int RECT_SIZE = REGION_SIZE * 5;
int XY_RATIO = 1;
int XY_SPLIT = 8;
int timedown = 0;
int vectormult = 1;




void calc_derivatives(Mat currFrame, Mat prevFrame, Mat &dx, Mat &dy, Mat &dt)
{
	dx.create(currFrame.rows - 1, currFrame.cols - 1, CV_64F);
	dy.create(currFrame.rows - 1, currFrame.cols - 1, CV_64F);
	dt.create(currFrame.rows - 1, currFrame.cols - 1, CV_64F);

	//CALC DERIVATIES
	for (int i = 0; i < currFrame.rows - 1; i++) {
		for (int j = 0; j < currFrame.cols - 1; j++) {
			double a, b, c, d;
			// Get dx gradient by calculating difference between frames
			a = currFrame.at<uchar>(i, j + 1) - currFrame.at<uchar>(i, j);
			b = currFrame.at<uchar>(i + 1, j + 1) - currFrame.at<uchar>(i + 1, j);
			c = prevFrame.at<uchar>(i, j + 1) - prevFrame.at<uchar>(i, j);
			d = prevFrame.at<uchar>(i + 1, j + 1) - prevFrame.at<uchar>(i + 1, j);
			dx.at<double>(i, j) = (a + b + c + d) / 4;

			// Get dy gradient by calculating difference between frames
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
	//blur(dt, dt, Size(3, 3));
	//blur(dx, dx, Size(2, 2));
	//blur(dx, dx, Size(2, 2));

	/*
	normalize(dx, dx, 0, 255, NORM_MINMAX);
	normalize(dy, dy, 0, 255, NORM_MINMAX);
	normalize(dt, dt, 0, 255, NORM_MINMAX);
	imshow("px", dx);
	imshow("py", dy);
	imshow("pt", dt);
	*/
}

void on_trackbar(int, void*){}

void createTrackbars(){
	namedWindow("bar", 0);
	char name[100];
	sprintf(name, "RegionSize", REGION_SIZE);
	sprintf(name, "MagFilter", MAGNITUDE_FILTER);
	sprintf(name, "XY_SPLIT", XY_SPLIT);
	sprintf(name, "XY_RATIO", XY_RATIO);
	sprintf(name, "RectSize", RECT_SIZE);
	sprintf(name, "vectormult", vectormult);

	createTrackbar("region_size", "bar", &REGION_SIZE, 50, on_trackbar);
	createTrackbar("magnitude filte", "bar", &MAGNITUDE_FILTER, 30, on_trackbar);
	createTrackbar("XY_SPLIT", "bar", &XY_SPLIT, 100, on_trackbar);
	createTrackbar("XY_RATIO", "bar", &XY_RATIO, 10, on_trackbar);
	createTrackbar("RECT_SIZE", "bar", &RECT_SIZE, REGION_SIZE*100, on_trackbar);
	createTrackbar("vectorsize", "bar", &vectormult, 15, on_trackbar);

}

void displayLRX(int row, int col, vector<Point2i> positions, vector<Vec2d> velocities, Mat &frame)
{
	Mat_<double> x_component(row, col, 0.5);
	Mat_<unsigned char> motionToLeft(row, col, UCHAR_MAX + 1);
	Mat_<unsigned char> motionToRight(row, col, UCHAR_MAX + 1);
	Mat_<unsigned char> motionDirection(row, col, UCHAR_MAX + 1);

	

	cout << velocities.size() << endl;
	//normalize(vels, vels, 0, 1, NORM_MINMAX);
	double sumX = 0;
	double sumY = 0;
	
	for (int i = 0; i < velocities.size(); i++) {
		Vec2d vel = velocities.at(i);
		double Vx = vel(0);
		double Vy = vel(1);
		sumX += vel(0);
		sumY += vel(1);

		//velocity is atleast 4 either way, bigger the value more confident we are,
		if (abs(Vx) > XY_SPLIT && abs(Vx) / abs(Vy) > XY_RATIO)  {
			if (Vx > 0 ) 
				rectangle(motionToRight, Rect(positions.at(i).x, positions.at(i).y, RECT_SIZE, RECT_SIZE), Scalar(255), CV_FILLED);
			else
				rectangle(motionToLeft, Rect(positions.at(i).x, positions.at(i).y, RECT_SIZE, RECT_SIZE), Scalar(255), CV_FILLED);
		}
		
			
	}

	if (abs(sumX)>XY_SPLIT && abs(sumX) / abs(sumY) >XY_RATIO){
		if (sumX > 0) //right
			rectangle(motionDirection, Rect(col / 2, 0, col, row), Scalar(255), CV_FILLED);
		else  //left
			rectangle(motionDirection, Rect(0, 0, col / 2, row), Scalar(255), CV_FILLED);
	}
	else if (abs(sumY) > XY_SPLIT && abs(sumY) / abs(sumX) > XY_RATIO) {
		if (sumY<0) //top
			rectangle(motionDirection, Rect(0, 0, col, row / 2), Scalar(255), CV_FILLED);
		else
			rectangle(motionDirection, Rect(0, row / 2, col, row), Scalar(255), CV_FILLED);
	}

	if (timedown == 0) {
		imshow("Motion Directiobn", motionDirection);
		timedown = 150;
	}
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
			
			//CALCULATE START AND END POINTS OF VECTORS
			Point centre, vector, start;
			centre.x = x+midshift;
			centre.y = y+midshift;
			vector.x = centre.x + (abs(Vx) > REGION_SIZE ? REGION_SIZE * vectormult : Vx * vectormult);
			vector.y = centre.y + (abs(Vy) > REGION_SIZE ? REGION_SIZE * vectormult : Vy * vectormult);
			start.x = x;
			start.y = y;

			double magnitude = sqrt(Vx*Vx + Vy*Vy);
			//only add usefull vectors
			if (magnitude > MAGNITUDE_FILTER) 	{
				line(frame, centre, vector, Scalar(0, 255, 255), 1, CV_AA);
				circle(frame, vector, 1, Scalar(255, 0, 0), 2);
				positions.push_back(start);
				velocities.push_back(vel);
			}

		}
	}

	displayLRX(frame.rows, frame.cols, positions, velocities, frame);
}


int main(int argc, const char** argv)
{
	cv::VideoCapture cap;
	Mat currentFrame, bwFrame, previousFrame, dx, dy, dt;

	//VIDEO CAPTURE
    /* Make it possibe to choose other connected webcams */
    long srcid = 0;
    char *endptr;
	if (argc > 1) {
        errno = 0;
        srcid = strtol(argv[1],&endptr,10);

        if ((errno == ERANGE && (srcid == LONG_MAX || srcid == LONG_MIN))
                || (errno != 0 && srcid == 0)) {
            perror("strtol");
            exit(EXIT_FAILURE);
        }
        if (endptr == argv[1]) {
            cap.open(string(argv[1]));
        } else {
            cap.open(srcid);
        }
    } else {
        cap.open(CV_CAP_ANY);
    }
	if (!cap.isOpened()) printf("Error: could not load a camera or video.\n");
	createTrackbars();
	cap >> currentFrame;
	cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);
	currentFrame.copyTo(previousFrame);

	//CREATE DX,DY,DT SO WE CAN CHECK SIZE LATER

	//RUN LOOP
	for (;;) {

		//~60fps
		waitKey(16);
		//get frame from video capture
		cap >> currentFrame;
		if (timedown > 0) {
			timedown -= 16;
			if (timedown < 0)
				timedown = 0;
		}
		if (!currentFrame.data) {
			printf("Error: no frame data.\n");
			break;
		}
		//flip horizontally so it does not look inverted
		flip(currentFrame, currentFrame, 1);
		cvtColor(currentFrame, bwFrame, CV_BGR2GRAY);
		equalizeHist(bwFrame, bwFrame);

		//COMPUTE DERIVATIVES
		calc_derivatives(bwFrame, previousFrame, dx, dy, dt);
		//COMPUTE LUCAS K ALGORITHM & DISPLAY MOTION
		LKTracker(dx, dy, dt, currentFrame);

		bwFrame.copyTo(previousFrame);
		imshow("video", currentFrame);			// Display content of frameOriginal
	}

	return 0;
}
