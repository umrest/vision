#pragma once

#include <vector>
#include <opencv2/opencv.hpp>


class CameraCalibration
{
	public:
		CameraCalibration();

		void RunCalibration(std::vector<cv::Mat> imgs);

	private:
		std::vector< std::vector< cv::Point3f > > object_points;
		std::vector< std::vector< cv::Point2f > > image_points;
		std::vector< cv::Point2f > corners;
		std::vector<std::vector< cv::Point2f > > left_img_points;

		cv::Mat img, gray;
		cv::Size im_size;

		int board_width = 7, board_height = 5;
		float square_size = 30;

		// Helper Functions
		void setup_calibration(std::vector<cv::Mat> &imgs);
		double computeReprojectionErrors(
			const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
			const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

};

