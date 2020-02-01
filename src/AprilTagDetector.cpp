#include "AprilTagDetector.h"

#include "math.h"

using namespace std;
using namespace cv;

AprilTagDetector::AprilTagDetector(double fx_in, double fy_in, double cx_in, double cy_in) : fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in),
																							 td(apriltag_detector_create()), tf(tagStandard41h12_create())
{

	apriltag_detector_add_family(td, tf);

	//td->quad_decimate = 2;
	//td->quad_sigma = 0.8;
	//td->refine_edges = 1;
	td->nthreads = 2;
}

AprilTagDetector::~AprilTagDetector()
{
	apriltag_detector_destroy(td);
	tagStandard41h12_destroy(tf);
}

void AprilTagDetector::detect(Mat &gray)
{

	vision.tag0.x = 0;
	vision.tag0.y = 0;
	vision.tag0.z = 0;
	vision.tag0.yaw = 0;
	vision.tag0.pitch = 0;
	vision.tag0.roll = 0;

	vision.tag1.x = 0;
	vision.tag1.y = 0;
	vision.tag1.z = 0;
	vision.tag1.yaw = 0;
	vision.tag1.pitch = 0;
	vision.tag1.roll = 0;

	image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

	// DETECTION

	zarray_t *detections = apriltag_detector_detect(td, &im);

	for (int i = 0; i < zarray_size(detections); i++)
	{

		apriltag_detection_t *det;
		zarray_get(detections, i, &det);

		// Do something with det here

		// Display detection box
		/*
		line(img, Point(det->p[0][0], det->p[0][1]),
			Point(det->p[1][0], det->p[1][1]),
			Scalar(0, 0xff, 0), 2);
		line(img, Point(det->p[0][0], det->p[0][1]),
			Point(det->p[3][0], det->p[3][1]),
			Scalar(0, 0, 0xff), 2);
		line(img, Point(det->p[1][0], det->p[1][1]),
			Point(det->p[2][0], det->p[2][1]),
			Scalar(0xff, 0, 0), 2);
		line(img, Point(det->p[2][0], det->p[2][1]),
			Point(det->p[3][0], det->p[3][1]),
			Scalar(0xff, 0, 0), 2);
*/

		// POS ESTIMATION

		// First create an apriltag_detection_info_t struct using your known parameters.
		apriltag_detection_info_t info;
		info.det = det;
		info.tagsize = .1345;
		info.fx = fx;
		info.fy = fy;
		info.cx = cx;
		info.cy = cy;

		// Then call estimate_tag_pose.
		apriltag_pose_t pose;
		double err = estimate_tag_pose(&info, &pose);

		comm::TagPosition *position;
		if (det->id == 0)
		{
			position = &vision.tag0;
		}
		if (det->id == 1)
		{
			position = &vision.tag1;
		}

		position->x = pose.t->data[0] * 39.3701; // * 86.6595 - .621;
		// flip x
		position->x = -position->x;

		position->y = pose.t->data[1] * 39.3701; // * 86.6595 - .621;
		position->z = pose.t->data[2] * 39.3701; // * 86.6595 - .621;

		double r11 = pose.R->data[0];
		double r12 = pose.R->data[1];
		double r13 = pose.R->data[2];
		double r21 = pose.R->data[3];
		double r22 = pose.R->data[4];
		double r23 = pose.R->data[5];
		double r31 = pose.R->data[6];
		double r32 = pose.R->data[7];
		double r33 = pose.R->data[8];

		position->roll = atan2(r21, r11) / (2 * 3.14) * 360;

		position->yaw = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) / (2 * 3.14) * 360;

		// flip y
		position->yaw = -position->yaw;

		position->pitch = atan2(r32, r33) / (2 * 3.14) * 360;
	}

	float tag_distance = 23.5; //inches
	float tag0_theta = vision.tag0.yaw * (2 * 3.14) / 360.0;
	float tag1_theta = vision.tag1.yaw * (2 * 3.14) / 360.0;

	float tag0_x_1 = sin(tag0_theta) * vision.tag0.z;
	float tag0_x_2 = sin(3.14 / 2.0 - tag0_theta) * vision.tag0.x;

	float tag0_y_1 = cos(tag0_theta) * vision.tag0.z;
	float tag0_y_2 = cos(3.14 / 2.0 - tag0_theta) * vision.tag0.x;

	float tag0_x_estimate = tag0_x_1 + tag0_x_2 - tag_distance / 2.0;
	float tag0_y_estimate = tag0_y_1 + tag0_y_2;
	float tag0_yaw_estimate = -vision.tag0.yaw;

	float tag1_x_1 = sin(tag1_theta) * vision.tag1.z;
	float tag1_x_2 = sin(3.14 / 2.0 - tag1_theta) * vision.tag1.x;

	float tag1_y_1 = cos(tag1_theta) * vision.tag1.z;
	float tag1_y_2 = cos(3.14 / 2.0 - tag1_theta) * vision.tag1.x;

	float tag1_x_estimate = tag1_x_1 + tag1_x_2 + tag_distance / 2.0;
	float tag1_y_estimate = tag1_y_1 + tag1_y_2;
	float tag1_yaw_estimate = -vision.tag1.yaw;

	// average the two tags
	if (vision.tag0.z != 0 && vision.tag1.z != 0)
	{
		vision.field_position.x = (tag0_x_estimate + tag1_x_estimate) / 2.0;
		vision.field_position.y = (tag0_y_estimate + tag1_y_estimate) / 2.0;
		vision.field_position.yaw = (tag0_yaw_estimate + tag1_yaw_estimate) / 2.0;
	}
	else if (vision.tag0.z != 0)
	{
		vision.field_position.x = tag0_x_estimate;
		vision.field_position.y = tag0_y_estimate;
		vision.field_position.yaw = tag0_yaw_estimate;
	}
	else if (vision.tag1.z != 0)
	{
		vision.field_position.x = tag1_x_estimate;
		vision.field_position.y = tag1_y_estimate;
		vision.field_position.yaw = tag1_yaw_estimate;
	}
	else
	{
		vision.field_position.x = 0;
		vision.field_position.y = 0;
		vision.field_position.yaw = 0;
	}
}

std::ostream &operator<<(std::ostream &os, const comm::TagPosition &t)
{
	os << t.x << " " << t.y << " " << t.z << " ";
	os << t.yaw << " " << t.pitch << " " << t.roll << "\r\n"
	   << std::flush;
	return os;
}
