#pragma once

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>

#include <opencv2/opencv.hpp>

#include <comm/Vision.h>
#include "Timer.h"

class AprilTagDetector
{

public:
	AprilTagDetector(double fx, double fy, double cx, double cy);
	~AprilTagDetector();
	void get_position_from_det(std::vector<apriltag_detection_t*> dets, comm::Tag_Position *position);

	void detect(cv::Mat &img);

	comm::Vision vision;
	int index = 0;
	Timer timer;

	cv::Mat rvec, tvec;
private:
	double fx, fy, cx, cy;

	apriltag_detector_t *td;
	apriltag_family_t *tf;

	cv::Mat gray;

	comm::Field_Position cur_fp;
	comm::Field_Position last_fp;
};

std::ostream &operator<<(std::ostream &os, const comm::Tag_Position &t);