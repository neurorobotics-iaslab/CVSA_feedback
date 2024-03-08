#include <ros/ros.h>
#include "feedback_cvsa/ControlCVSA.h"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "controlcvsa");

	ros::NodeHandle nh;
	feedback::ControlCVSA cvsa;

	if(cvsa.configure() == false) {
		ROS_ERROR("ControlCVSA configuration failed");
		ros::shutdown();
		return 0;
	}


	cvsa.run();

	ros::shutdown();

	return 0;
}
