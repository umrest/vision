#include "PositionROSPublisher.h"

PositionROSPublisher::PositionROSPublisher(int argc, char* argv[]) {
	ros::init(argc, argv, "talker");

	pub = n.advertise<std_msgs::String>("vision_position", 1000);

}

bool PositionROSPublisher::publish(TagPosition data)
{
	if (!ros::ok()) { return false; }

	std_msgs::String msg;

	std::stringstream ss;
	ss << data.x << " " << data.y << " " << data.z << " " << data.pitch << " " << data.roll << " " << data.yaw;

	msg.data = ss.str();

	pub.publish(msg);

	std::cout << ss.str() << std::endl;

	return true;
}
