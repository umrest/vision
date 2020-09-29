#include "AprilTagDetector.h"

#include "math.h"
#include <map>

using namespace std;
using namespace cv;
using namespace comm;

#define pi 3.141592653589

AprilTagDetector::AprilTagDetector(double fx_in, double fy_in, double cx_in, double cy_in) : fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in),
																							 td(apriltag_detector_create()), tf(tag16h5_create())
{

	apriltag_detector_add_family(td, tf);

	td->quad_decimate = 4;
	//td->quad_sigma = 0.8;
	td->refine_edges = 0;
	td->nthreads = 2;

	vision._field_position.set_x(0);
	vision._field_position.set_y(0);
	vision._field_position.set_yaw(0);
}

AprilTagDetector::~AprilTagDetector()
{
	apriltag_detector_destroy(td);
	tag16h5_destroy(tf);
}

void AprilTagDetector::get_position_from_det(std::vector<apriltag_detection_t*> dets, Tag_Position *position)
{
	std::map<int, Point2d> tag_center_offsets;
	// -x -> left, +y -> up
	tag_center_offsets[0] = cv::Point2d(-.310,0); // tag 0 is in the middle at 0,0
	tag_center_offsets[1] = cv::Point2d(0,0); // tag 1 is in the left at -1,0
	tag_center_offsets[2] = cv::Point2d(.315,0); // tag 2 is in the rights at 1,0
	tag_center_offsets[3] = cv::Point2d(-.310,.247); // tag 2 is in the rights at 1,0
	tag_center_offsets[4] = cv::Point2d(0,.247); // tag 2 is in the rights at 1,0
	tag_center_offsets[5] = cv::Point2d(.315,.247); // tag 2 is in the rights at 1,0

	double tagsize = .152;
	vector<Point3d> object_points;
	vector<Point2d> image_points;

	int valid_tag_count = 0;

	for(apriltag_detection_t* det : dets){
		if(tag_center_offsets.count(det->id)){
			valid_tag_count++;
			cv::Point2d offset = tag_center_offsets[det->id];

			object_points.push_back(cv::Point3d(-tagsize/2 + offset.y, -tagsize/2 + offset.x, 0));
			object_points.push_back(cv::Point3d(-tagsize/2 + offset.y, tagsize/2 + offset.x, 0));
			object_points.push_back(cv::Point3d(tagsize/2 + offset.y, tagsize/2 + offset.x, 0));
			object_points.push_back(cv::Point3d(tagsize/2 + offset.y, -tagsize/2 + offset.x, 0));
				
			image_points.push_back(cv::Point2d(det->p[0][0], det->p[0][1]));
			image_points.push_back(cv::Point2d(det->p[1][0], det->p[1][1]));
			image_points.push_back(cv::Point2d(det->p[2][0], det->p[2][1]));
			image_points.push_back(cv::Point2d(det->p[3][0], det->p[3][1]));
		}
	}

	std::cout << "Localizing off of " << valid_tag_count << " tags." << std::endl;

	//std::cout << object_points.size() << std::endl;

	if(valid_tag_count > 2){
		Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
		Mat distCoeffs = (Mat1d(4, 1) << 0, 0, 0, 0);

		
		bool starting_guess = !rvec.empty() && !tvec.empty();
		cv::solvePnP(object_points, image_points, cameraMatrix, distCoeffs, rvec, tvec, starting_guess);

		cv::Mat R;
		cv::Rodrigues(rvec, R);

		R = R.t();          // inverse rotation
		tvec = -R * tvec;   // translation of inverse

		// camPose is a 4x4 matrix with the pose of the camera in the object frame
		cv::Mat camPose = cv::Mat::eye(4, 4, R.type());
		R.copyTo(camPose.rowRange(0, 3).colRange(0, 3)); // copies R into camPose
		tvec.copyTo(camPose.rowRange(0, 3).colRange(3, 4)); // copies tvec into camPose

		double pitch = atan2(-camPose.at<double>(2,1), camPose.at<double>(2,2));
		double yaw = asin(camPose.at<double>(2,0));
		double roll = atan2(-camPose.at<double>(1,0), camPose.at<double>(0,0));

		double scale = 90.0 / (pi / 2);
		position->set_yaw(yaw * scale);
		position->set_pitch(pitch * scale);
		position->set_roll(roll * scale);
		
		position->set_x(camPose.at<double>(1, 3) * 39.3701); // * 86.6595 - .621;
		position->set_y(camPose.at<double>(0, 3) * 39.3701);	 // * 86.6595 - .621;
		position->set_z(-camPose.at<double>(2, 3) * 39.3701);	 // * 86.6595 - .621;
	}
}


void AprilTagDetector::detect(Mat &gray)
{
	image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

	// DETECTION
	timer.reset();
	zarray_t *detections = apriltag_detector_detect(td, &im);
	//std::cout << timer.elapsed() << "s to detect tags" << std::endl;

	timer.reset();
	comm::Tag_Position tag0_pos1;
	comm::Tag_Position tag1_pos1;

	// reset tag poses
	tag0_pos1.set_x( 0);
	tag0_pos1.set_y ( 0);
	tag0_pos1.set_z ( 0);
	tag0_pos1.set_yaw ( 0);
	tag1_pos1.set_x ( 0);
	tag1_pos1.set_y ( 0);
	tag1_pos1.set_z ( 0);
	tag1_pos1.set_yaw ( 0);

	bool pose_valid = false;

	std::vector<apriltag_detection_t*> dets;


	for (int i = 0; i < zarray_size(detections); i++)
	{
		pose_valid = true;

		apriltag_detection_t *det;
		zarray_get(detections, i, &det);

		if(det->hamming < 1){
			dets.push_back(det);
		}

		
	}
	get_position_from_det(dets, &vision._tag0);
	apriltag_detections_destroy(detections);

	vision.set_vision_good( pose_valid ? 1 : 0);
	//std::cout << timer.elapsed() << "s to process tags" << std::endl;
}
