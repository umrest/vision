#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>

#include "CameraCalibration.h"

using namespace std;
using namespace cv;

CameraCalibration::CameraCalibration() {

}

void CameraCalibration::RunCalibration(std::vector<cv::Mat> imgs) {

	setup_calibration(imgs);

	printf("Starting Calibration\n");
	Mat K;
	Mat D;
	vector< Mat > rvecs, tvecs;
	int flag = 0;
	flag |= CV_CALIB_FIX_K4;
	flag |= CV_CALIB_FIX_K5;
	calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

	cout << "Calibration error: " << computeReprojectionErrors(rvecs, tvecs, K, D) << endl;

	cout << K.at<double>(0,1) << endl;
	cout << K.at<double>(0, 0) << ", ";
	cout << K.at<double>(1, 1) << ", ";
	cout << K.at<double>(0, 2) << ", ";
	cout << K.at<double>(1, 2) << endl;

	printf("Done Calibration\n");
}

void CameraCalibration::setup_calibration(std::vector<cv::Mat> &imgs) {
	Size board_size = Size(board_width, board_height);
	int board_n = board_width * board_height;

	for (int i = 0; i < imgs.size(); i++) {
		img = imgs[i];
		cv::cvtColor(img, gray, CV_BGR2GRAY);

		bool found = false;
		found = cv::findChessboardCorners(img, board_size, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray, board_size, corners, found);
		}

		vector< Point3f > obj;
		for (int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found) {
			cout << i << ". Found corners!" << endl;
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
		else {
			cout << "Cannot find corners" << endl;
		}
	}
}

double CameraCalibration::computeReprojectionErrors(
	const vector< Mat >& rvecs, const vector< Mat >& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs) {
	vector< Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	vector< float > perViewErrors;
	perViewErrors.resize(object_points.size());

	for (i = 0; i < (int)object_points.size(); ++i) {
		projectPoints(Mat(object_points[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(image_points[i]), Mat(imagePoints2), CV_L2);
		int n = (int)object_points[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}
