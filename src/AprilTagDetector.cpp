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

	vision._field_position.set_x(0);
	vision._field_position.set_y(0);
	vision._field_position.set_yaw(0);

	moving_average.set_x( 0);
	moving_average.set_y( 0);
	moving_average.set_yaw( 0);
}

AprilTagDetector::~AprilTagDetector()
{
	apriltag_detector_destroy(td);
	tagStandard41h12_destroy(tf);
}

void get_position_from_pose(apriltag_pose_t pose, TagPosition *position)
{
	position->set_x(-pose.t->data[0] * 39.3701); // * 86.6595 - .621;

	position->set_y(pose.t->data[1] * 39.3701); // * 86.6595 - .621;
	position->set_z(pose.t->data[2] * 39.3701); // * 86.6595 - .621;

	double r11 = pose.R->data[0];
	double r12 = pose.R->data[1];
	double r13 = pose.R->data[2];
	double r21 = pose.R->data[3];
	double r22 = pose.R->data[4];
	double r23 = pose.R->data[5];
	double r31 = pose.R->data[6];
	double r32 = pose.R->data[7];
	double r33 = pose.R->data[8];

	position->set_roll(atan2(r21, r11) / (2 * 3.14) * 360);

	position->set_yaw (-atan2(-r31, sqrt(r32 * r32 + r33 * r33)) / (2 * 3.14) * 360);



	position->set_pitch(atan2(r32, r33) / (2 * 3.14) * 360);
}

void get_fieldposition_estimate(TagPosition tag, float tag_displacement, FieldPosition *fp)
{
	float tag_distance = 23.5; // displacement from center
	float tag_theta = -tag.get_yaw() * (2 * 3.14) / 360.0;

	float tag_x_1 = sin(tag_theta) * tag.get_z();
	float tag_x_2 = sin(3.14 / 2.0 - tag_theta) * tag.get_x();

	float tag_y_1 = cos(-tag_theta) * tag.get_z();
	float tag_y_2 = cos(3.14 / 2.0 + tag_theta) * tag.get_x();

	float tag_x_estimate = tag_x_1 + tag_x_2 - tag_displacement;
	float tag_y_estimate = tag_y_1 + tag_y_2;
	float tag_yaw_estimate = tag.get_yaw();

	fp->set_x ( tag_x_estimate);
	fp->set_y( tag_y_estimate);
	fp->set_yaw ( tag_yaw_estimate);
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
	tag0_pos1.set_x( 0);
	tag0_pos1.set_y ( 0);
	tag0_pos1.set_z ( 0);
	tag0_pos1.set_yaw ( 0);
	tag1_pos1.set_x ( 0);
	tag1_pos1.set_y ( 0);
	tag1_pos1.set_z ( 0);
	tag1_pos1.set_yaw ( 0);
	tag0_pos2.set_x ( 0);
	tag0_pos2.set_y ( 0);
	tag0_pos2.set_z ( 0);
	tag0_pos2.set_yaw ( 0);
	tag1_pos2.set_x ( 0);
	tag1_pos2.set_y ( 0);
	tag1_pos2.set_z ( 0);
	tag1_pos2.set_yaw ( 0);

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

	//cout << "0: " << tag0_pos1_fp.set_x( << " " << tag0_pos2_fp.set_x( << endl;
	// cout << "1: " << tag1_pos1_fp.set_x( << " " << tag1_pos2_fp.set_x( << endl;

	// try all pairs of tags and poses to find best match

	// pose1_2 -> tag0pos1, tag1_pos2
	int pose1_1 = abs(tag0_pos1_fp.get_x() - tag1_pos1_fp.get_x()) + abs(tag0_pos1_fp.get_y() - tag1_pos1_fp.get_y());
	int pose1_2 = abs(tag0_pos1_fp.get_x() - tag1_pos2_fp.get_x()) + abs(tag0_pos1_fp.get_y() - tag1_pos2_fp.get_y());
	int pose2_1 = abs(tag0_pos2_fp.get_x() - tag1_pos1_fp.get_x()) + abs(tag0_pos2_fp.get_y() - tag1_pos1_fp.get_y());
	int pose2_2 = abs(tag0_pos2_fp.get_x() - tag1_pos2_fp.get_x()) + abs(tag0_pos2_fp.get_y() - tag1_pos2_fp.get_y());

	std::vector<int> pose_errors = {pose1_1, pose1_2, pose2_1};
	vision._tag0.set_x( tag0_pos1.get_x());
	vision._tag0.set_y ( tag0_pos1.get_y());
	vision._tag0.set_z( tag0_pos1.get_z());
	vision._tag0.set_yaw( tag0_pos1.get_yaw());
	vision._tag1.set_x( tag1_pos1.get_x());
	vision._tag1.set_y (tag1_pos1.get_y());
	vision._tag1.set_z ( tag1_pos1.get_z());
	vision._tag1.set_yaw ( tag1_pos1.get_yaw());

	vision._field_position.set_x ( 0);
		vision._field_position.set_y (   0);
		vision._field_position.set_yaw ( 0);

	

	

	// both tags are visible
	if (tag0_pos1.get_z() != 0 && tag1_pos1.get_z() != 0)
	{
		int idx = std::min_element(pose_errors.begin(), pose_errors.end()) - pose_errors.begin();
		//cout << idx << endl;
			//cout << "0: " << tag0_pos1_fp.set_x( << " " << tag0_pos2_fp.set_x( << endl;
			//cout << "1: " << tag1_pos1_fp.set_x( << " " << tag1_pos2_fp.set_x( << endl;
			
			// pose1_1 had min error
			if (idx == 0)
			{
				cur_fp.set_x(  (tag0_pos1_fp.get_x() + tag1_pos1_fp.get_x()) / 2.0);
				cur_fp.set_y(  (tag0_pos1_fp.get_y() + tag1_pos1_fp.get_y()) / 2.0);
				cur_fp.set_yaw( (tag0_pos1_fp.get_yaw() + tag1_pos1_fp.get_yaw()) / 2.0);
			}
			// pose1_2
			else if (idx == 1)
			{
				cur_fp.set_x(  (tag0_pos1_fp.get_x() + tag1_pos2_fp.get_x()) / 2.0);
				cur_fp.set_y(  (tag0_pos1_fp.get_y() + tag1_pos2_fp.get_y()) / 2.0);
				cur_fp.set_yaw( (tag0_pos1_fp.get_yaw() + tag1_pos2_fp.get_yaw()) / 2.0);
			}
			// pose2_1
			else if (idx == 2)
			{
				cur_fp.set_x(  (tag0_pos2_fp.get_x() + tag1_pos1_fp.get_x()) / 2.0);
				cur_fp.set_y(  (tag0_pos2_fp.get_y() + tag1_pos1_fp.get_y()) / 2.0);
				cur_fp.set_yaw( (tag0_pos2_fp.get_yaw() + tag1_pos1_fp.get_yaw()) / 2.0);
			}
	
		
	}
	else if (vision._tag0.get_z() != 0)
	{
		cur_fp.set_x(  tag0_pos1_fp.get_x());
		cur_fp.set_y(  tag0_pos1_fp.get_y());
		cur_fp.set_yaw( tag0_pos1_fp.get_yaw());
	}
	else if (vision._tag1.get_z() != 0)
	{
		cur_fp.set_x( tag1_pos1_fp.get_x());
		cur_fp.set_y(  tag1_pos1_fp.get_y());
		cur_fp.set_yaw(  tag1_pos1_fp.get_yaw());
	}
	else
	{
		vision._tag0.set_x(0);
		vision._tag0.set_y(  0);
		vision._tag0.set_z(  0);
		vision._tag0.set_yaw(  0);
		vision._tag1.set_x  (0);
		vision._tag1.set_y  (0);
		vision._tag1.set_z  (0);
		vision._tag1.set_yaw(  0);
	}
	
	if(pose_valid){
		
		int diff = abs(moving_average.get_x() - cur_fp.get_x()) + abs(moving_average.get_y() - cur_fp.get_y());

		if(diff < 30){
			moving_average.set_x ( (alpha * cur_fp.get_x() + (1.0 - alpha) * moving_average.get_x()));
			moving_average.set_y ( (alpha * cur_fp.get_y() + (1.0 - alpha) * moving_average.get_y()));
			moving_average.set_yaw( (alpha * cur_fp.get_yaw() + (1.0 - alpha) * moving_average.get_yaw()));
			reset_times = 0;
		}
		else if(reset_times > 5){
			
			moving_average.set_x( cur_fp.get_x());
			moving_average.set_y( cur_fp.get_y());
			moving_average.set_yaw (cur_fp.get_yaw());
		}
		else{
			reset_times++;
		}

		vision._field_position.set_x(   moving_average.get_x());
		vision._field_position.set_y(    moving_average.get_y());
		vision._field_position.set_yaw ( moving_average.get_yaw());
	}

	vision.set_vision_good( pose_valid ? 1 : 0);
}

std::ostream &operator<<(std::ostream &os, const comm::TagPosition &t)
{
	//os << t.get_x() << " " << t.get_y() << " " << t.get_z() << " ";
	//os << t.get_yaw() << " " << t.get_pitch() << " " << t.get_roll() << "\r\n"
	//   << std::flush;
	return os;
}
