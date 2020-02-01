#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CameraCalibration.h"
#include "AprilTagDetector.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

#include "TcpClient.h"

//#include "PositionROSPublisher.h"

using namespace std;
using namespace cv;

cv::Mat img;
cv::VideoCapture cap;

std::mutex mtx;

void camera_worker()
{
	Mat tmp;
	while (true)
	{
		if (!cap.read(tmp))
		{
			break;
		}
		std::lock_guard<std::mutex> lock(mtx);
		img = tmp;
	}

	cout << "Unable to capture from video device" << endl;
	cap.release();
}

int main(int argc, char *argv[])
{

	cap.open(0); //"/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0");

	int resolution_x = 640;
	int resolution_y = 480;

	double fx = (resolution_x / 640) * 354.961;
	double fy = (resolution_y / 480) * 354.961;
	double cx = 320.961;
	double cy = 249.12;

	cap.set(CAP_PROP_FRAME_WIDTH, resolution_x);
	cap.set(CAP_PROP_FRAME_HEIGHT, resolution_y);

	if (argc > 1)
	{
		std::string arg = argv[1];
		cout << arg << endl;
		if (arg == "calibrate")
		{
			cout << "Run Calibration " << endl;
			vector<cv::Mat> imgs(15);

			for (int i = 0; i < 15; i++)
			{
				std::ostringstream ss;

				ss << "calibration01/calib_" << i << ".jpg";

				imgs[i] = cv::imread(ss.str());
			}

			CameraCalibration calib;
			calib.RunCalibration(imgs);
		}
		if (arg == "capture")
		{
			cout << "Capture Calibration Images" << endl;
			// Calibration part

			cout << cap.set(CAP_PROP_AUTO_EXPOSURE, .75);

			cv::Mat img;

			cap.read(img);

			//cv::imshow("Image", img);

			//cv::waitKey();

			for (int i = 0; i < 15; i++)
			{
				cout << i << endl;
				while (cv::waitKey(100) == 255)
				{
					cap.read(img);
					cv::imshow("Image", img);
				}

				std::ostringstream ss;

				ss << "calibration01/calib_" << i << ".jpg";

				cv::imwrite(ss.str(), img);
			}

			cap.release();
		}
		return 0;
	}

	// Detection part

	// Webcam
	//AprilTagDetector det(614.659, 584.008, 323.213, 51.519);

	// USB Camera

	//PositionROSPublisher pub(argc, argv);
	AprilTagDetector det(fx, fy, cx, cy);

	cout << cap.set(CAP_PROP_AUTO_EXPOSURE, .75);
	//cout << cap.set(CAP_PROP_EXPOSURE, .025);

	//cout << cap.set(CAP_PROP_CONVERT_RGB , false);
	//cout << cap.set(CAP_PROP_FOURCC, CV_FOURCC('Y','U','Y','V') );

	std::thread camera_worker_thread(camera_worker);
	while (img.empty())
	{
		std::cout << "Camera not ready..." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	std::vector<uchar> buffer(4095);

	boost::asio::io_service io_service;

	TcpClient client;

	double quality = 10;

	char recv[128];

	Mat gray;

	while (true)
	{

		if (true)
		{
			std::lock_guard<std::mutex> lock(mtx);

			cvtColor(img, gray, cv::COLOR_RGB2GRAY);
		}

		if (client.read_nonblocking(recv, 128))
		{

			cout << (int)recv[0] << endl;
			if (recv[0] == (char)comm::CommunicationDefinitions::TYPE::VISION_COMMAND)
			{
				if (true)
				{
					cout << "Sending image:";

					// send image to dashboard
					std::vector<int> param(2);
					param[0] = cv::IMWRITE_JPEG_QUALITY;
					param[1] = 5;

					cv::imencode(".jpeg", gray, buffer, param);
					cout << buffer.size() << endl;

					/*
					comm::Vis command;
					command.command = 

					auto type = (comm::CommunicationDefinitions::TYPE)recv[0];
					int sz = comm::CommunicationDefinitions::PACKET_SIZES.at(type) + 1;
					char buf[sz];
					buf[0] = (char)comm::CommunicationDefinitions::TYPE::VISION_IMAGE;
					std::copy(buffer.begin(), buffer.begin() + sz - 1, buf + 1);
					client.write(buf, sz);
					*/
				}
				else if (true)
				{
					cout << "Setting Vision Properties" << endl;
					double exposure = *(short *)(recv + 1) / 100.0;
					int gain = (int)recv[5];
					cout << exposure << " ";
					cout << gain << " ";
					cout << cap.set(CAP_PROP_AUTO_EXPOSURE, .75);
					cout << cap.set(CAP_PROP_EXPOSURE, exposure);
					cout << cap.set(CAP_PROP_GAIN, gain);
					cout << endl;
				}
			}
		}

		det.detect(gray);

		//cout << det.t1 << endl;

		//cv::imshow("Captured", img);
		//cv::waitKey(1);

		client.write(det.vision.Serialize());

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}
