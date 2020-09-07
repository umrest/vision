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

#include <comm/RESTClient.h>

#include <comm/Vision.h>
#include <comm/Vision_Image.h>
#include <comm/Identifier.h>
#include <comm/Vision_Command.h>

using namespace std;
using namespace cv;

class VisionMain
{
	cv::Mat img;
	cv::VideoCapture cap;

	std::mutex mtx;
	std::vector<uchar> buffer;

	comm::RESTClient client;

	int auto_quality = 2;
	bool streaming_enabled = false;
	bool detection_enabled = true;

	void send_image(Mat &img)
	{
		// send image to dashboard
		std::vector<int> param(2);
		param[0] = cv::IMWRITE_JPEG_QUALITY;
		param[1] = auto_quality;
		cv::imencode(".jpg", img, buffer, param);

		if(buffer.size() > 8191 * 0.8){
			auto_quality -= 3;
		}
		else{
			auto_quality += 1;
		}
		buffer.resize(8191);


		std::unique_ptr<comm::RESTPacket> vision_image_packet(new comm::Vision_Image);

		((comm::Vision_Image*)vision_image_packet.get())->set_image((char*)&buffer[0]);
		client.send_message(vision_image_packet.get());

	}

	void camera_worker()
	{
		Mat tmp;
		while(true){	
			if(cap.open("/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0")){
				while (true)
				{
					if (!cap.read(tmp))
					{
						break;
					}
					std::lock_guard<std::mutex> lock(mtx);
					img = tmp;
				}
			}

			cout << "Unable to capture from video device" << endl;
			cap.release();
		}
	}

public:
	VisionMain() : client(comm::CommunicationDefinitions::IDENTIFIER::VISION)
	{
	}
	void run()
	{
		

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

		double angle = 0;

		bool enable_streaming = false;
		bool enable_detection = true;

		while (true)
		{
		if(true){
					std::lock_guard<std::mutex> lock(mtx);

					cvtColor(img, gray, cv::COLOR_RGB2GRAY);
		}

			if(enable_detection){
				det.detect(gray);
				client.send_message(&det.vision);
			}
			
			if (enable_streaming)
			{
				send_image(gray);
			}

		
			auto messages = client.get_messages();
			for(auto &m : messages){
				if(m->type() == comm::CommunicationDefinitions::TYPE::VISION_COMMAND){
					comm::Vision_Command* command = (comm::Vision_Command*)m.get();
					if(command->get_command() == 5){
						send_image(gray);
						enable_streaming = false;
					}
					if(command->get_command() == 6){
						enable_streaming = true;
					}
					if(command->get_command() == 7){
						enable_streaming = false;
					}

					if(command->get_command() == 8){
						enable_detection = true;
					}
					if(command->get_command() == 9){
						enable_detection = false;
					}
				}
			}
			


		}
	}
};

int main(int argc, char *argv[])
{

	VisionMain v;
	v.run();
}
