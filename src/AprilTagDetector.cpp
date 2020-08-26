#include "AprilTagDetector.h"

#include "math.h"

using namespace std;
using namespace cv;
using namespace comm;

AprilTagDetector::AprilTagDetector(double fx_in, double fy_in, double cx_in, double cy_in) : fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in),
																							 td(apriltag_detector_create()), tf(tagStandard41h12_create())
{

	apriltag_detector_add_family(td, tf);

	//td->quad_decimate = 2;
	//td->quad_sigma = 0.8;
	//td->refine_edges = 1;
	td->nthreads = 2;

	vision.field_position.x = 0;
	vision.field_position.y = 0;
	vision.field_position.yaw = 0;

	moving_average.x = 0;
	moving_average.y = 0;
	moving_average.yaw = 0;
}

AprilTagDetector::~AprilTagDetector()
{
	apriltag_detector_destroy(td);
	tagStandard41h12_destroy(tf);
}

void get_position_from_pose(apriltag_pose_t pose, TagPosition *position)
{
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

void get_fieldposition_estimate(TagPosition tag, float tag_displacement, FieldPosition *fp)
{
	float tag_distance = 23.5; // displacement from center
	float tag_theta = -tag.yaw * (2 * 3.14) / 360.0;

	float tag_x_1 = sin(tag_theta) * tag.z;
	float tag_x_2 = sin(3.14 / 2.0 - tag_theta) * tag.x;

	float tag_y_1 = cos(-tag_theta) * tag.z;
	float tag_y_2 = cos(3.14 / 2.0 + tag_theta) * tag.x;

	float tag_x_estimate = tag_x_1 + tag_x_2 - tag_displacement;
	float tag_y_estimate = tag_y_1 + tag_y_2;
	float tag_yaw_estimate = tag.yaw;

	fp->x = tag_x_estimate;
	fp->y = tag_y_estimate;
	fp->yaw = tag_yaw_estimate;
}

void AprilTagDetector::detect(Mat &gray)
{
	image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

	// DETECTION

	zarray_t *detections = apriltag_detector_detect(td, &im);

	comm::TagPosition tag0_pos1;
	comm::TagPosition tag0_pos2;
	comm::TagPosition tag1_pos1;
	comm::TagPosition tag1_pos2;

	// reset tag poses
	tag0_pos1.x = 0;
	tag0_pos1.y = 0;
	tag0_pos1.z = 0;
	tag0_pos1.yaw = 0;
	tag1_pos1.x = 0;
	tag1_pos1.y = 0;
	tag1_pos1.z = 0;
	tag1_pos1.yaw = 0;
	tag0_pos2.x = 0;
	tag0_pos2.y = 0;
	tag0_pos2.z = 0;
	tag0_pos2.yaw = 0;
	tag1_pos2.x = 0;
	tag1_pos2.y = 0;
	tag1_pos2.z = 0;
	tag1_pos2.yaw = 0;

		bool pose_valid = false;

	for (int i = 0; i < zarray_size(detections); i++)
	{
		pose_valid = true;

		apriltag_detection_t *det;
		zarray_get(detections, i, &det);

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
		apriltag_pose_t pose1;
		apriltag_pose_t pose2;
		double err1;
		double err2;
		estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, 50);

		if (det->id == 0)
		{
			get_position_from_pose(pose1, &tag0_pos1);
			if (err2 != HUGE_VAL)
			{
				get_position_from_pose(pose2, &tag0_pos2);
			}
		}
		else if (det->id == 1)
		{
			get_position_from_pose(pose1, &tag1_pos1);
			if (err2 != HUGE_VAL)
			{
				get_position_from_pose(pose2, &tag1_pos2);
			}
		}
	}
	apriltag_detections_destroy(detections);

	FieldPosition tag0_pos1_fp;
	FieldPosition tag0_pos2_fp;
	FieldPosition tag1_pos1_fp;
	FieldPosition tag1_pos2_fp;
	get_fieldposition_estimate(tag0_pos1, 24.0 / 2.0, &tag0_pos1_fp);
	get_fieldposition_estimate(tag0_pos2, 24 / 2.0, &tag0_pos2_fp);
	get_fieldposition_estimate(tag1_pos1, -24.0 / 2.0, &tag1_pos1_fp);
	get_fieldposition_estimate(tag1_pos2, -24 / 2.0, &tag1_pos2_fp);

	//cout << "0: " << tag0_pos1_fp.x << " " << tag0_pos2_fp.x << endl;
	// cout << "1: " << tag1_pos1_fp.x << " " << tag1_pos2_fp.x << endl;

	// try all pairs of tags and poses to find best match

	// pose1_2 -> tag0pos1, tag1_pos2
	int pose1_1 = abs(tag0_pos1_fp.x - tag1_pos1_fp.x) + abs(tag0_pos1_fp.y - tag1_pos1_fp.y);
	int pose1_2 = abs(tag0_pos1_fp.x - tag1_pos2_fp.x) + abs(tag0_pos1_fp.y - tag1_pos2_fp.y);
	int pose2_1 = abs(tag0_pos2_fp.x - tag1_pos1_fp.x) + abs(tag0_pos2_fp.y - tag1_pos1_fp.y);
	int pose2_2 = abs(tag0_pos2_fp.x - tag1_pos2_fp.x) + abs(tag0_pos2_fp.y - tag1_pos2_fp.y);

	std::vector<int> pose_errors = {pose1_1, pose1_2, pose2_1};
	vision.tag0.x = tag0_pos1.x;
	vision.tag0.y = tag0_pos1.y;
	vision.tag0.z = tag0_pos1.z;
	vision.tag0.yaw = tag0_pos1.yaw;
	vision.tag1.x = tag1_pos1.x;
	vision.tag1.y = tag1_pos1.y;
	vision.tag1.z = tag1_pos1.z;
	vision.tag1.yaw = tag1_pos1.yaw;

	vision.field_position.x =   0;
		vision.field_position.y =   0;
		vision.field_position.yaw = 0;

	

	

	// both tags are visible
	if (tag0_pos1.z != 0 && tag1_pos1.z != 0)
	{
		int idx = std::min_element(pose_errors.begin(), pose_errors.end()) - pose_errors.begin();
		//cout << idx << endl;
			//cout << "0: " << tag0_pos1_fp.x << " " << tag0_pos2_fp.x << endl;
			//cout << "1: " << tag1_pos1_fp.x << " " << tag1_pos2_fp.x << endl;
			
			// pose1_1 had min error
			if (idx == 0)
			{
				cur_fp.x = (tag0_pos1_fp.x + tag1_pos1_fp.x) / 2.0;
				cur_fp.y = (tag0_pos1_fp.y + tag1_pos1_fp.y) / 2.0;
				cur_fp.yaw = (tag0_pos1_fp.yaw + tag1_pos1_fp.yaw) / 2.0;
			}
			// pose1_2
			else if (idx == 1)
			{
				cur_fp.x = (tag0_pos1_fp.x + tag1_pos2_fp.x) / 2.0;
				cur_fp.y = (tag0_pos1_fp.y + tag1_pos2_fp.y) / 2.0;
				cur_fp.yaw = (tag0_pos1_fp.yaw + tag1_pos2_fp.yaw) / 2.0;
			}
			// pose2_1
			else if (idx == 2)
			{
				cur_fp.x = (tag0_pos2_fp.x + tag1_pos1_fp.x) / 2.0;
				cur_fp.y = (tag0_pos2_fp.y + tag1_pos1_fp.y) / 2.0;
				cur_fp.yaw = (tag0_pos2_fp.yaw + tag1_pos1_fp.yaw) / 2.0;
			}
	
		
	}
	else if (vision.tag0.z != 0)
	{
		cur_fp.x = tag0_pos1_fp.x;
		cur_fp.y = tag0_pos1_fp.y;
		cur_fp.yaw = tag0_pos1_fp.yaw;
	}
	else if (vision.tag1.z != 0)
	{
		cur_fp.x = tag1_pos1_fp.x;
		cur_fp.y = tag1_pos1_fp.y;
		cur_fp.yaw = tag1_pos1_fp.yaw;
	}
	else
	{
		vision.tag0.x = 0;
		vision.tag0.y = 0;
		vision.tag0.z = 0;
		vision.tag0.yaw = 0;
		vision.tag1.x = 0;
		vision.tag1.y = 0;
		vision.tag1.z = 0;
		vision.tag1.yaw = 0;
	}
	
	if(pose_valid){
		
		int diff = abs(moving_average.x - cur_fp.x) + abs(moving_average.y - cur_fp.y);

		if(diff < 30){
			moving_average.x = (alpha * cur_fp.x) + (1.0 - alpha) * moving_average.x;
			moving_average.y = (alpha * cur_fp.y) + (1.0 - alpha) * moving_average.y;
			moving_average.yaw = (alpha * cur_fp.yaw) + (1.0 - alpha) * moving_average.yaw;
			reset_times = 0;
		}
		else if(reset_times > 5){
			
			moving_average.x = cur_fp.x;
			moving_average.y = cur_fp.y;
			moving_average.yaw = cur_fp.yaw;
		}
		else{
			reset_times++;
		}

		vision.field_position.x =   moving_average.x;
		vision.field_position.y =   moving_average.y;
		vision.field_position.yaw = moving_average.yaw;
	}

	vision.vision_good = pose_valid ? 1 : 0;
}

std::ostream &operator<<(std::ostream &os, const comm::TagPosition &t)
{
	os << t.x << " " << t.y << " " << t.z << " ";
	os << t.yaw << " " << t.pitch << " " << t.roll << "\r\n"
	   << std::flush;
	return os;
}
