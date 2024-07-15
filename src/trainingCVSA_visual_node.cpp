#include <ros/ros.h>
#include "feedback_cvsa/TrainingCVSA_visual.h"


int main(int argc, char** argv) {

    // ros initialization
    ros::init(argc, argv, "trainingCVSA_visual_node");

    feedback::TrainingCVSA_visual cvsa;

    if(cvsa.configure() == false) {
        ROS_ERROR("TrainingCVSA configuration failed");
        ros::shutdown();
        return 0;
    }


    cvsa.run();

    ros::shutdown();

    return 0;
    
    
}
