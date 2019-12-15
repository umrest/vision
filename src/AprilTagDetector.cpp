#include "AprilTagDetector.h"

#include "math.h"

using namespace std;
using namespace cv;

AprilTagDetector::AprilTagDetector(double fx_in, double fy_in, double cx_in, double cy_in) :
	fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in),
	td(apriltag_detector_create()), tf(tagStandard41h12_create()) {

	apriltag_detector_add_family(td, tf);

	td->quad_decimate = 2;
	td->quad_sigma = 0.25;
	td->refine_edges = 1;


}

AprilTagDetector::~AprilTagDetector() {
	apriltag_detector_destroy(td);
	tagStandard41h12_destroy(tf);
}


void AprilTagDetector::detect(Mat &img) {

	//t0.reset();
	//t1.reset();


	Mat gray;

	cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };


	// DETECTION

	zarray_t* detections = apriltag_detector_detect(td, &im);


	for (int i = 0; i < zarray_size(detections); i++) {

		apriltag_detection_t* det;
		zarray_get(detections, i, &det);

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
		info.tagsize = .213;
		info.fx = fx;
		info.fy = fy;
		info.cx = cx;
		info.cy = cy;

		// Then call estimate_tag_pose.
		apriltag_pose_t pose;
		double err = estimate_tag_pose(&info, &pose);

		TagPosition position;

		position.x = pose.t->data[0] * 39.3701;// * 86.6595 - .621;
		position.y = pose.t->data[1] * 39.3701;// * 86.6595 - .621;
		position.z = pose.t->data[2] * 39.3701;// * 86.6595 - .621;

		double r11 = pose.R->data[0];
		double r12 = pose.R->data[1];
		double r13 = pose.R->data[2];
		double r21 = pose.R->data[3];
		double r22 = pose.R->data[4];
		double r23 = pose.R->data[5];
		double r31 = pose.R->data[6];
		double r32 = pose.R->data[7];
		double r33 = pose.R->data[8];


		position.roll = atan2(r21, r11) / (2*3.14) * 360;

		position.yaw = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) / (2 * 3.14) * 360;

		position.pitch = atan2(r32, r33) / (2 * 3.14) * 360;

		if(det->id == 0){
			t0 = position;
		}
		if(det->id == 1){
			t1 = position;
		}
		
	}

	apriltag_detections_destroy(detections);
}

std::ostream& operator<<(std::ostream& os, const TagPosition& t)
{
    os << t.x << " " << t.y << " " << t.z << "\n";
	os << t.yaw << " " << t.pitch << " " << t.roll << "\n";
    return os;
}
