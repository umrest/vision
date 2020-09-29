#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ArucoTagDetector.h"
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
#include <comm/Vision_Properties.h>

#include "Timer.h"

using namespace std;
using namespace cv;

class VisionMain
{
	cv::Mat gray;
	cv::VideoCapture cap;

	std::mutex mtx;
	std::vector<uchar> buffer;

	comm::RESTClient client;
	bool new_image = false;

	int auto_quality = 2;
	bool streaming_enabled = false;
	bool detection_enabled = true;

	double exposure = 0;
	double gain = 0;

	
	int resolution_x = 640; //800; // 2592
	int resolution_y = 480; //600; // 1944

	Timer timer;

	void send_image(Mat &img)
	{
		// send image to dashboard
		std::vector<int> param(2);
		param[0] = cv::IMWRITE_JPEG_QUALITY;
		param[1] = auto_quality;
		cv::imencode(".jpg", img, buffer, param);

		if (buffer.size() > 8191 * 0.8)
		{
			auto_quality -= 3;
		}
		else
		{
			auto_quality += 1;
		}
		buffer.resize(8191);
		if (auto_quality < 1)
		{
			auto_quality = 1;
		}

		std::unique_ptr<comm::RESTPacket> vision_image_packet(new comm::Vision_Image);

		((comm::Vision_Image *)vision_image_packet.get())->set_image((char *)&buffer[0]);
		client.send_message(vision_image_packet.get());
	}

	void set_exposure(){
		bool auto_exposure = (exposure == 0);
		std::cout << "exposure: auto: " << auto_exposure << " val: " << exposure << std::endl;
		std::cout << cap.get(CAP_PROP_AUTO_EXPOSURE) << std::endl;

		cap.set(CAP_PROP_EXPOSURE, exposure);
		cap.set(CAP_PROP_AUTO_EXPOSURE, auto_exposure ? 3 : 1);
		cap.set(CAP_PROP_GAIN, gain);
		cap.set(CAP_PROP_BUFFERSIZE, 1);

		std::cout << cap.get(CAP_PROP_AUTO_EXPOSURE) << std::endl;
		
	}

	void camera_worker()
	{
		Mat tmp;
		while (true)
		{
			if (cap.open("/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0"))
			{
				std::cout << cap.getBackendName() << std::endl;
				set_exposure();
				
				cap.set(CAP_PROP_FRAME_WIDTH, resolution_x);
				cap.set(CAP_PROP_FRAME_HEIGHT, resolution_y);
				
				//cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V') );
				//.set(CAP_PROP_CONVERT_RGB , false);
				cap.set(CAP_PROP_CONVERT_RGB , true);
				cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G') );

				while (true)
				{
					if (!cap.grab())
					{
						break;
					}
					Mat tmp;
					if(!cap.retrieve(tmp)){
						
						break;
					}
					//cvtColor(tmp, gray, cv::COLOR_YUV2GRAY_YUY2);
					cvtColor(tmp, gray, cv::COLOR_RGB2GRAY);
					new_image = true;
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

	void handle_messages(){
		auto messages = client.get_messages();
		for (auto &m : messages)
		{
			if (m->type() == comm::CommunicationDefinitions::TYPE::VISION_COMMAND)
			{
				comm::Vision_Command *command = (comm::Vision_Command *)m.get();
				if (command->get_command() == 5)
				{
					std::cout << "Got Send Image Command" << std::endl;
					send_image(gray);
					streaming_enabled = false;
				}
				if (command->get_command() == 6)
				{
					std::cout << "Got Send Enable Streaming Command" << std::endl;
					streaming_enabled = true;
				}
				if (command->get_command() == 7)
				{
					std::cout << "Got Send Disable Streaming Command" << std::endl;
					streaming_enabled = false;
				}

				if (command->get_command() == 8)
				{
					std::cout << "Got Send Enable Detection Command" << std::endl;
					detection_enabled = true;
				}
				if (command->get_command() == 9)
				{
					std::cout << "Got Send Disable Detection Command" << std::endl;
					detection_enabled = false;
				}
			}

			if (m->type() == comm::CommunicationDefinitions::TYPE::VISION_PROPERTIES)
			{
				std::cout << "Got Vision Properties" << std::endl;

				comm::Vision_Properties *properties = (comm::Vision_Properties *)m.get();

				exposure = properties->get_exposure();
				gain = properties->get_gain();
				set_exposure();
				
			}
		}
	}

	void process_image()
	{
		
		
		if (new_image)
		{
			//std::cout << timer.elapsed() << "s to get image" << std::endl;
		}
		else{
			return;
		}				

		if (detection_enabled)
		{
			std::lock_guard<std::mutex> lock(mtx);
			double fx = (resolution_x / 640.0) * 354.961;
			double fy = (resolution_y / 480.0) * 354.961;

			double cx = (resolution_x / 640.0) * 320.961;
			double cy = (resolution_y / 480.0) * 249.12;
			AprilTagDetector det(fx, fy, cx, cy);
			det.detect(gray);

			client.send_message(&det.vision);
		}

		if (streaming_enabled)
		{
			std::lock_guard<std::mutex> lock(mtx);
			send_image(gray);
		}

		if(new_image){
			new_image = false;
			timer.reset();
		}

		
	}
	void run()
	{
		boost::asio::io_service io_service;
		double quality = 10;
		double angle = 0;

std::thread camera_worker_thread(&VisionMain::camera_worker, this);
		
		while (true)
		{
			process_image();
			handle_messages();
		}
	}
};

int main(int argc, char *argv[])
{

	VisionMain v;
	v.run();
}
