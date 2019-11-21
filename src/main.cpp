#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "CameraCalibration.h"
#include "AprilTagDetector.h"
#include "Socket.h"

#include <iostream>
#include <chrono>
#include <thread>

#include "VisionData.h"


//#include "PositionROSPublisher.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{

	/*
	// Calibration part
	
	cv::VideoCapture cap;
	cv::Mat img;
	std::vector<cv::Mat> imgs;

	cap.open(0);
	cap.read(img);
	cap.release();

	cv::imshow("Image", img);
	cv::waitKey();

	for (int i = 0; i < 25; i++) {

		cv::waitKey(1000);

		cap.open(0);
		cap.read(img);
		cap.release();

		imgs.push_back(img);
		cv::imshow("Image", img);

		std::ostringstream ss;

		ss << "calibration01/calib_" << i << ".jpg";

		cv::imwrite(ss.str(), img);

	}

	CameraCalibration calib;
	calib.RunCalibration(imgs);
	
	*/

	// Detection part

	// Webcam
	//AprilTagDetector det(614.659, 584.008, 323.213, 51.519);

	// USB Camera

	//PositionROSPublisher pub(argc, argv);

	AprilTagDetector det(302.211, 300.491, 330.439, 241.819);

	cv::Mat img;
	cv::Mat gray;

	cv::VideoCapture cap;

	cap.set(CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CAP_PROP_FRAME_HEIGHT, 1080);
	//cap.set(CAP_PROP_EXPOSURE, 10000);

	cap.open(0);

	Socket s;


	while (true)
	{

		if (cap.read(img))
		{
			det.detect(img);

			cv::imshow("Captured", img);
			cv::waitKey(1);

			VisionData v(det.position);
			char* data = v.Serialize();
			s.send_data(data);
			
			if(!s.connected()){
				std::this_thread::sleep_for(2s);
			}
		}
		else
		{
			cout << "Unable to capture from video device" << endl;
			break;
		}
	}
}