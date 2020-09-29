#include "ArucoTagDetector.h"

#include "math.h"

using namespace std;
using namespace cv;
using namespace comm;

#define pi 3.141592653589

ArucoTagDetector::ArucoTagDetector(double fx_in, double fy_in, double cx_in, double cy_in) : fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in)
{
	double gap = 0.014;
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    board = cv::aruco::GridBoard::create(2, 2, 4*gap, gap, dictionary);

	vision._field_position.set_x(0);
	vision._field_position.set_y(0);
	vision._field_position.set_yaw(0);
}

ArucoTagDetector::~ArucoTagDetector()
{
}

void get_position_from_pose(Vec3d &rvec, Vec3d &tvec, Tag_Position *position)
{
	position->set_x(-tvec[0] * 39.3701); // * 86.6595 - .621;
	position->set_y(tvec[1] * 39.3701);	 // * 86.6595 - .621;
	position->set_z(tvec[2] * 39.3701);	 // * 86.6595 - .621;

	Mat R;
	cv::Rodrigues(rvec, R);

	double cos_beta = sqrt(R.at<double>(2,1) * R.at<double>(2,1) + R.at<double>(2,2) * R.at<double>(2,2));
    bool validity = cos_beta < 1e-6l;

	double alpha, beta, gamma;

    if (! validity){
        alpha = atan2(R.at<double>(1,0), R.at<double>(0,0));
        beta  = atan2(-R.at<double>(2,0), cos_beta);
        gamma = atan2(R.at<double>(2,1), R.at<double>(2,2)); 
	}
    else{
        alpha = atan2(R.at<double>(1,0), R.at<double>(0,0));
        beta  = atan2(-R.at<double>(2,0), cos_beta);
        gamma = 0   ;                      
	}

	double scale = 90.0 / (pi / 2);
	position->set_yaw(beta * scale);
	position->set_pitch(gamma * scale);
	position->set_roll(alpha * scale);
}

void ArucoTagDetector::detect(Mat &gray)
{
	comm::Tag_Position tag0_pos1;
	comm::Tag_Position tag1_pos1;

	// reset tag poses
	tag0_pos1.set_x(0);
	tag0_pos1.set_y(0);
	tag0_pos1.set_z(0);
	tag0_pos1.set_yaw(0);
	tag1_pos1.set_x(0);
	tag1_pos1.set_y(0);
	tag1_pos1.set_z(0);
	tag1_pos1.set_yaw(0);

	bool pose_valid = false;

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters, rejectedCandidates);

	Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	Mat distCoeffs = (Mat1d(4, 1) << 0, 0, 0, 0);
	std::vector<cv::Vec3d> rvecs, tvecs;

	std::cout << corners.size() << " ";
	
	bool valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvecs, tvecs);


	
	if(valid){
		for (int i = 0; i < ids.size(); i++)
			{
				std::cout << ids[i] <<  " ";
				pose_valid = true;

				if (ids[i] == 0)
				{
					get_position_from_pose(rvecs[i], tvecs[i], &vision._tag0);
				}
				else if (ids[i] == 1)
				{
					get_position_from_pose(rvecs[i], tvecs[i], &vision._tag1);
				}
			}
	}

	
	std::cout << std::endl;

	vision.set_vision_good(pose_valid ? 1 : 0);
}