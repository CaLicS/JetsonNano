#include "opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "dxl.hpp"
#include <signal.h>
#include <unistd.h>

using namespace cv;
using namespace std;

bool ctrl_c_pressed = false;
void ctrlc(int)
{
	ctrl_c_pressed = true;
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

	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
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
	}

	VideoCapture source("7_lt_ccw_100rpm_in.mp4");
	if (!source.isOpened())
	{
		cout << "video x" << endl;
	}

	Mat frame, labels, stats, centroids;
	vector<double> vx(2);
	vector<double> vy(2);
	Dxl mx;
	if (!mx.open())
	{
		cout << "dynamixel open error" << endl;
		return -1;
	}
	struct timeval start, end1;
	signal(SIGINT, ctrlc);

	bool start_con = false;
	bool first = true;

	double cur_min = 0;
	double min = 0;
	double time1 = 0;

	int vel1 = 0, vel2 = 0, error = 0;
	float gain = 0.3;

	Point2d bef_vec;
	while (true)
	{
		gettimeofday(&start, NULL);
		if (mx.kbhit())
		{
			char ch = mx.getch();
			if (ch == 's')
			{
				start_con = true;
			}
		}
		if (start_con)
		{
			min = 0;
			cur_min = 0;

			source >> frame;
			if (frame.empty())
			{
				cerr << "frame empty!" << endl;
				break;
			}
			writer1 << frame;

			Mat dst = frame(Rect(0, frame.rows / 4 * 3, frame.cols, 90));
			cvtColor(dst, dst, COLOR_BGR2GRAY);
			dst = dst + (100 - mean(dst)[0]);
			writer3 << dst;
			threshold(dst, dst, 130, 255, THRESH_BINARY);
			int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);
			cvtColor(dst, dst, COLOR_GRAY2BGR);
			

			for (int i = 1; i < cnt; i++)
			{
				double *p = centroids.ptr<double>(i);
				int *ps = stats.ptr<int>(i);

				rectangle(dst, Rect(ps[0], ps[1], ps[2], ps[3]), Scalar(255, 0, 0));

				if (first && (p[0] > 200 && p[0] < 440 && p[1] > 20))
				{
					vx.insert(vx.begin(), p[0]);
					vx.push_back(p[0]);

					vy.insert(vy.begin(), p[1]);
					vy.push_back(p[1]);

					first = false;
					cur_min = sqrt((p[0] - dst.cols / 2.0, 2) + pow(p[1] - dst.rows / 2.0, 2));
				}

				min = sqrt(pow(vx.back() - p[0], 2) + pow(vy.back() - p[1], 2));

				if (i == 1)
					cur_min = min;

				if ((abs(vx.back() - p[0]) <= 150 && abs(vy.back() - p[1] <= 60)) && (min <= cur_min))
				{
					cur_min = min;
					vx.insert(vx.begin(), p[0]);
					vy.insert(vy.begin(), p[1]);
					error = dst.cols / 2.0 - p[0];
					rectangle(dst, Rect(ps[0], ps[1], ps[2], ps[3]), Scalar(0, 0, 255));
				}
			}
			circle(dst, Point2d(vx.front(), vy.front()), 4, Scalar(0, 0, 255), -1);

			vel1 = 30 - gain * error;
			vel2 = -(30 + gain * error);

			vx.push_back(vx.front());
			vy.push_back(vy.front());

			if (!mx.setVelocity(vel1, vel2))
			{
				cout << "setVelocity error" << endl;
				return -1;
			}

			if (ctrl_c_pressed)
				break;

			usleep(20 * 1000);

			gettimeofday(&end1, NULL);
			time1 = end1.tv_sec - start.tv_sec +
					(end1.tv_usec - start.tv_usec) / 1000000.0;
			cout << "err: " << error << " vel1: " << vel1 << " vel2: " << vel2
				 << "time: " << time1 << endl;
			writer2 << dst;
			waitKey(15);
		}
	}
	mx.close();
	return 0;
}