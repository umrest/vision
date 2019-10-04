#include "AprilTagDetector.h"

#include "math.h"

using namespace std;
using namespace cv;

AprilTagDetector::AprilTagDetector(double fx_in, double fy_in, double cx_in, double cy_in) :
	fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in),
	td(apriltag_detector_create()), tf(tagStandard41h12_create()) {

	apriltag_detector_add_family(td, tf);

	td->quad_decimate = 2;
	td->quad_sigma = 0;
	td->refine_edges = 1;


}

AprilTagDetector::~AprilTagDetector() {
	apriltag_detector_destroy(td);
	tagStandard41h12_destroy(tf);
}


void AprilTagDetector::detect(Mat &img) {
	Mat gray;

	cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };


	// DETECTION

	zarray_t* detections = apriltag_detector_detect(td, &im);


	for (int i = 0; i < zarray_size(detections); i++) {

		apriltag_detection_t* det;
		zarray_get(detections, i, &det);

		cout << det->id << endl;

		// Do something with det here

		// Display detection box

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

		// POS ESTIMATION

		// First create an apriltag_detection_info_t struct using your known parameters.
		apriltag_detection_info_t info;
		info.det = det;
		info.tagsize = .12;
		info.fx = fx;
		info.fy = fy;
		info.cx = cx;
		info.cy = cy;

		// Then call estimate_tag_pose.
		apriltag_pose_t pose;
		double err = estimate_tag_pose(&info, &pose);

		position.x = pose.t->data[0];
		position.y = pose.t->data[1];
		position.z = pose.t->data[2];

		double r11 = pose.R->data[0];
		double r12 = pose.R->data[1];
		double r13 = pose.R->data[2];
		double r21 = pose.R->data[3];
		double r22 = pose.R->data[4];
		double r23 = pose.R->data[5];
		double r31 = pose.R->data[6];
		double r32 = pose.R->data[7];
		double r33 = pose.R->data[8];

		position.yaw = atan(r21 / r11) / (2*3.14) * 360;

		position.pitch = atan(-r31 / sqrt(r32 * r32 + r33 * r33)) / (2 * 3.14) * 360;

		position.roll = atan(r32 / r33) / (2 * 3.14) * 360;
		
	}

	apriltag_detections_destroy(detections);
}