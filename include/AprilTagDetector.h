#pragma once

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>

#include <opencv2/opencv.hpp>

#include "TagPosition.h"

class AprilTagDetector
{

public:
	AprilTagDetector(double fx, double fy, double cx, double cy);
	~AprilTagDetector();

	void detect(cv::Mat &img);

	TagPosition t0;
	TagPosition t1;

private:

	double fx, fy, cx, cy;

	apriltag_detector_t* td;
	apriltag_family_t* tf;
};

