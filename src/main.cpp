#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CameraCalibration.h"
#include "AprilTagDetector.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>

#include "TcpClient.h"

#include <comm/Vision.h>
#include <comm/Identifier.h>

//#include "PositionROSPublisher.h"

using namespace std;
using namespace cv;

class VisionMain
{
	cv::Mat img;
	cv::VideoCapture cap;

	std::mutex mtx;
	std::vector<uchar> buffer;

	TcpClient client;

	bool streaming_enabled = false;
	bool detection_enabled = true;
	float auto_quality = 10;

	void send_image(Mat &img)
	{
		// send image to dashboard
		std::vector<int> param(2);
		param[0] = cv::IMWRITE_JPEG_QUALITY;
		param[1] = auto_quality;
		cv::imencode(".jpg", img, buffer, param);
		cout << "Sending Image: " << buffer.size() << endl;

		vector<uint8_t> type(1);
		type[0] = (uint8_t)comm::CommunicationDefinitions::TYPE::VISION_IMAGE;
		int target_sz = 8000;
		int sz = comm::CommunicationDefinitions::PACKET_SIZES.at(comm::CommunicationDefinitions::TYPE::VISION_IMAGE) + 1;
		cout << sz << endl;
		int diff = target_sz-buffer.size();
                cout << diff << endl;
		auto_quality+=diff/400.0;

		if(auto_quality < 1){auto_quality =1;}
		if(auto_quality > 100){auto_quality=100;}
                cout << auto_quality << endl;
		std::vector<uchar> buf(sz);
		buf[0] = (char)comm::CommunicationDefinitions::TYPE::VISION_IMAGE;
		std::copy(buffer.begin(), buffer.begin() + sz, buf.begin() + 1);
		client.write(buf);

		
	}

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

public:
	VisionMain()
	{
	}
	void run()
	{
		cap.open("/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0"); /*usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0*/

		int resolution_x = 640;
		int resolution_y = 480;

		double fx = (resolution_x / 640) * 354.961;
		double fy = (resolution_y / 480) * 354.961;
		double cx = 320.961;
		double cy = 249.12;

		cap.set(CAP_PROP_FRAME_WIDTH, resolution_x);
		cap.set(CAP_PROP_FRAME_HEIGHT, resolution_y);

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

		std::thread camera_worker_thread(&VisionMain::camera_worker, this);
		while (img.empty())
		{
			std::cout << "Camera not ready..." << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		boost::asio::io_service io_service;

		double quality = 10;

		uint8_t recv[128];

		Mat gray;

		while (true)
		{

			if (true)
			{
				std::lock_guard<std::mutex> lock(mtx);

				cvtColor(img, gray, cv::COLOR_RGB2GRAY);
			}
			if (streaming_enabled)
			{
				send_image(gray);
			}

			if (client.read_nonblocking(recv, 3))
			{
				bool valid_key = true;
				for (int i = 0; i < 3; i++)
				{
					if (recv[i] != comm::CommunicationDefinitions::key[i])
					{
						valid_key = false;
					}
				}

				if (valid_key)
				{
					cout << "valid Key" << endl;

					if (client.read_nonblocking(recv, 128))
					{
						cout << (int)recv[0] << endl;
						if (recv[0] == (char)comm::CommunicationDefinitions::TYPE::VISION_COMMAND)
						{
							if (recv[1] == 0)
							{
								cout << "SEnding image" << endl;
								send_image(gray);
							}
						}
						else if (recv[0] == (char)comm::CommunicationDefinitions::TYPE::VISION_PROPERTIES)
						{
							if (recv[1] == 1)
							{
								cout << "Setting Vision Properties" << endl;
							}
							else if (recv[1] == 2)
							{

								streaming_enabled = recv[2] == 1;
								detection_enabled = recv[3] == 1;

								cout << "Setting Vision State" << endl;
							}
						}
				}
			}
		}
		cout << detection_enabled << endl;
		if (detection_enabled)
		{

			det.detect(gray);

			//std::cout << det.vision.tag0 << std::endl;

			//cv::imshow("Captured", img);
			//cv::waitKey(1);
			auto data = det.vision.Serialize();

			client.write(data);

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}
}
;

int main(int argc, char *argv[])
{

	VisionMain v;
	v.run();
}
