#include "opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
#include <time.h>
//#include <sys/time.h>
//#include "dxl.hpp"
#include <signal.h>
//#include <unistd.h>

using namespace cv;
using namespace std;

bool ctrl_c_pressed = false;
void ctrlc(int)
{
	ctrl_c_pressed = true;
}
void vec(vector<double>* v, double cen, int con)
{
	v->insert(v->begin(), cen);
	if(con)
		v->push_back(cen);
}
int main(void)
{
	// string src = "nvarguscamerasrc sensor-id=0 ! \
	// 	video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
	// 	format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
	// 	width=(int)640, height=(int)360, format=(string)BGRx ! \
	// 	videoconvert ! video/x-raw, format=(string)BGR ! appsink";
	// VideoCapture source(src, CAP_GSTREAMER);
	// if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }

	/*string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
		nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
		h264parse ! rtph264pay pt=96 ! \
		udpsink host=192.168.0.70 port=8111 sync=false";
	VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
	if (!writer1.isOpened())
	{
		cerr << "Writer open failed!" << endl;
		return -1;
	}

	string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
		nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
		h264parse ! rtph264pay pt=96 ! \
		udpsink host=192.168.0.70 port=8112 sync=false";
	VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);
	if (!writer2.isOpened())
	{
		cerr << "Writer open failed!" << endl;
		return -1;
	}

	string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
		nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
		h264parse ! rtph264pay pt=96 ! \
		udpsink host=192.168.0.70 port=8113 sync=false";
	VideoWriter writer3(dst3, 0, (double)30, Size(640, 90), false);
	if (!writer2.isOpened())
	{
		cerr << "Writer open failed!" << endl;
		return -1;
	}*/

	VideoCapture source("lanefollow_100rpm_ccw.mp4");
	if (!source.isOpened())
	{
		cout << "video x" << endl;
	}

	Mat frame, labels, stats, centroids;
	vector<double> vx1(2);
	vector<double> vy1(2);
	vector<double> vx2(2);
	vector<double> vy2(2);
	//Dxl mx;
	/*if (!mx.open())
	{
		cout << "dynamixel open error" << endl;
		return -1;
	}*/
	//struct timeval start, end1;
	signal(SIGINT, ctrlc);

	bool start_con = true;
	bool first1 = true;
	bool first2 = true;
	bool test = true;

	double cur_min1 = 0;
	double cur_min2 = 0;
	double min1 = 0;
	double min2 = 0;
	double time1 = 0;

	int vel1 = 0, vel2 = 0, error = 0;
	float gain = 0.3;

	Point2d bef_vec;
	while (true)
	{
		//gettimeofday(&start, NULL);
		/*if (mx.kbhit())
		{
			char ch = mx.getch();
			if (ch == 's')
			{
				start_con = true;
			}
			else if(ch == 'q')
				break;
		}*/
		if (start_con)
		{
			min1 = 0;
			min2 = 0;
			cur_min1 = 0;
			cur_min2 = 0;

			source >> frame;
			if (frame.empty())
			{
				cerr << "frame empty!" << endl;
				break;
			}
			//writer1 << frame;

			Mat dst = frame(Rect(0, frame.rows / 4 * 3, frame.cols, 90));
			cvtColor(dst, dst, COLOR_BGR2GRAY);
			dst = dst + (100 - mean(dst)[0]);
			//writer3 << dst;
			threshold(dst, dst, 150, 255, THRESH_BINARY);
			int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);
			cvtColor(dst, dst, COLOR_GRAY2BGR);


			for (int i = 1; i < cnt; i++)
			{
				double* p = centroids.ptr<double>(i);
				int* ps = stats.ptr<int>(i);

				rectangle(dst, Rect(ps[0], ps[1], ps[2], ps[3]), Scalar(255, 0, 0));

				if (first1 && (p[0] > 440 && p[1] > 20))
				{
					vec(&vx1, p[0], 1);
					vec(&vy1, p[1], 1);

					first1 = false;
					cur_min1 = sqrt((p[0] - dst.cols / 2.0, 2) + pow(p[1] - dst.rows / 2.0, 2));
				}
				if (first2 && (p[0] < 200 && p[1] > 20))
				{
					vec(&vx2, p[0], 1);
					vec(&vy2, p[1], 1);

					first2 = false;
					cur_min2 = sqrt((p[0] - dst.cols / 2.0, 2) + pow(p[1] - dst.rows / 2.0, 2));
				}

				min1 = sqrt(pow(vx1.back() - p[0], 2) + pow(vy1.back() - p[1], 2));
				min2 = sqrt(pow(vx2.back() - p[0], 2) + pow(vy2.back() - p[1], 2));

				if (i == 1)
					cur_min1 = min1, cur_min2 = min2;

				if ((abs(vx1.back() - p[0]) <= 70 && abs(vy1.back() - p[1] <= 60)) && (min1 <= cur_min1))
				{
					cur_min1 = min1;
					vec(&vx1, p[0], 0);
					vec(&vy1, p[1], 0);
					rectangle(dst, Rect(ps[0], ps[1], ps[2], ps[3]), Scalar(0, 0, 255));
				}
				if ((abs(vx2.back() - p[0]) <= 70 && abs(vy2.back() - p[1] <= 60)) && (min2 <= cur_min2))
				{
					cur_min2 = min2;
					vec(&vx2, p[0], 0);
					vec(&vy2, p[1], 0);
					rectangle(dst, Rect(ps[0], ps[1], ps[2], ps[3]), Scalar(255, 0, 255));
				}
			}
			error = (dst.cols / 2.0) - ((vx1.front() + vx2.front()) / 2.0);
			circle(dst, Point2d(vx1.front(), vy1.front()), 4, Scalar(0, 0, 255), -1);
			circle(dst, Point2d(vx2.front(), vy2.front()), 4, Scalar(0, 0, 255), -1);

			vel1 = 30 - gain * error;
			vel2 = -(30 + gain * error);

			vx1.push_back(vx1.front());
			vy1.push_back(vy1.front());
			vx2.push_back(vx2.front());
			vy2.push_back(vy2.front());

			/*if (!mx.setVelocity(vel1, vel2))
			{
				cout << "setVelocity error" << endl;
				return -1;
			}*/

			if (ctrl_c_pressed)
				break;

			//usleep(20 * 1000);

			//gettimeofday(&end1, NULL);
			//time1 = end1.tv_sec - start.tv_sec +
			//	(end1.tv_usec - start.tv_usec) / 1000000.0;
			cout << "err: " << error << " vel1: " << vel1 << " vel2: " << vel2
				<< "time: " << time1 << endl;
			//writer2 << dst;
			imshow("dst", dst);
			imshow("frame", frame);

			waitKey(15);
		}
	}
	//mx.close();
	return 0;
}