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
	cv::VideoCapture cap;
	cap.open("/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0");

	int resolution_x = 640;
	int resolution_y = 480;

	double fx = (resolution_x / 640) * 354.961;
	double fy = (resolution_y / 480) * 354.961;
	double cx = 320.961;
	double cy = 249.12;

	cout << cap.set(CAP_PROP_FRAME_WIDTH, resolution_x);
	cout << cap.set(CAP_PROP_FRAME_HEIGHT,resolution_y);
	
	cout << cap.set(CAP_PROP_AUTO_EXPOSURE, .25);
	cout << cap.set(CAP_PROP_EXPOSURE, .025);
	cout << endl;


	if(argc > 1){
		std::string arg = argv[1];
		cout << arg << endl;
		if(arg == "calibrate"){
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
		if(arg == "capture"){
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
				while(cv::waitKey(100) == 255){
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
	AprilTagDetector det( fx ,  fy , cx, cy);

	cv::Mat img;
	cv::Mat gray;

	std::vector<uchar> buffer(4095);

	Socket s;

	double quality = 10;

	while (true)
	{
		

		if (cap.read(img))
		{
			

			char recv[128];
			if(s.recieve_data(recv)){
				cout << "Sending image" << endl;
				
				// send image to dashboard
				std::vector<int> param(2);
				param[0] = cv::IMWRITE_JPEG_QUALITY;
				param[1] = 60;

				cv::imencode(".jpeg", img, buffer, param);
				
				char buf[65536];
				buf[0] = 13;
				std::copy(buffer.begin(), buffer.begin() + 65536-1, buf+1);
				s.send_data(buf, 65536);
			}

			det.detect(img);

			cout << det.t0 << endl;

			//cv::imshow("Captured", img);
			//cv::waitKey(1);

			VisionData v(det.t0, det.t1);
			char *data = v.Serialize();
			s.send_data(data);
			
			//std::this_thread::sleep_for(100ms);

			// Wait 2 seconds before trying to reconnect
			if (!s.connected())
			{
				cout << "Not connected" << endl;
				std::this_thread::sleep_for(std::chrono::seconds(2));
			}
		}
		else
		{
			cout << "Unable to capture from video device" << endl;
			break;
		}
	}
}
