#pragma once

#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/apriltag_pose.h>

#include <opencv2/opencv.hpp>

#include <comm/Vision.h>

class AprilTagDetector
{

public:
	AprilTagDetector(double fx, double fy, double cx, double cy);
	~AprilTagDetector();

	void detect(cv::Mat &img);

	comm::Vision vision;

private:
	double fx, fy, cx, cy;

	apriltag_detector_t *td;
	apriltag_family_t *tf;

	cv::Mat gray;

	comm::FieldPosition cur_fp;
	comm::FieldPosition last_fp;
	comm::FieldPosition moving_average;
	// closer to 0 -> more averaging
	double alpha = 0.3;
	int reset_times = 0;
};

std::ostream &operator<<(std::ostream &os, const comm::TagPosition &t);