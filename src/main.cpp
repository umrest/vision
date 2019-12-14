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

	cout << cap.set(CAP_PROP_FRAME_WIDTH, 480);
	cout << cap.set(CAP_PROP_FRAME_HEIGHT,320);
	
	cout << cap.set(CAP_PROP_AUTO_EXPOSURE, .75);
//	cout << cap.set(CAP_PROP_EXPOSURE, .025);
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
			cv::imshow("Image", img);

			cv::waitKey();

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
	AprilTagDetector det( 159.911 ,  159.911 , 364.816, 231.372);

	cv::Mat img;
	cv::Mat gray;

	

	Socket s;

	while (true)
	{

		if (cap.read(img))
		{
			det.detect(img);

			cout << det.t0 << endl;

//			cv::imshow("Captured", img);/
//			cv::waitKey(1);

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
