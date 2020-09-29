#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <comm/Vision.h>

class ArucoTagDetector
{

public:
	ArucoTagDetector(double fx, double fy, double cx, double cy);
	~ArucoTagDetector();

	void detect(cv::Mat &img);

	comm::Vision vision;

private:
	double fx, fy, cx, cy;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::GridBoard> board;

	cv::Mat gray;

	comm::Field_Position cur_fp;
	comm::Field_Position last_fp;
};

std::ostream &operator<<(std::ostream &os, const comm::Tag_Position &t);