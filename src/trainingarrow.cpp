#include <ros/ros.h>
#include "rosneuro_feedback_wheel/TrainingWheel.h"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "trainingwheel");

	rosneuro::feedback::TrainingWheel wheel;

	if(wheel.configure() == false) {
		ROS_ERROR("TrainingWheel configuration failed");
		ros::shutdown();
		return 0;
	}


	wheel.run();

	ros::shutdown();

	return 0;
}
