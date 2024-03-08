#include <ros/ros.h>
#include "feedback_cvsa/TrainingCVSA.h"


int main(int argc, char** argv) {

    // ros initialization
    ros::init(argc, argv, "trainingCVSA_node");

    feedback::TrainingCVSA cvsa;

    if(cvsa.configure() == false) {
        ROS_ERROR("TrainingCVSA configuration failed");
        ros::shutdown();
        return 0;
    }


    cvsa.run();

    ros::shutdown();

    return 0;
    
    
}
