#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "TagPosition.h"

class PositionROSPublisher
{
public:
	PositionROSPublisher(int argc, char* argv[]);

	bool publish(TagPosition data);
private:
	ros::NodeHandle n;

	ros::Publisher pub;
};

