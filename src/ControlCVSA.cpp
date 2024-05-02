#include "feedback_cvsa/ControlCVSA.h"

namespace feedback {

ControlCVSA::ControlCVSA(void) : CVSA_layout("controlCVSA"), p_nh_("~") {

	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
	this->sub_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &ControlCVSA::on_received_data, this);

}

ControlCVSA::~ControlCVSA(void) {}

bool ControlCVSA::configure(void) {
	// Getting classes
    if(this->p_nh_.getParam("classes", this->classes_) == false) {
        ROS_ERROR("Parameter 'classes' is mandatory");
        return false;
    } 
    this->set_nclasses(this->classes_.size());

	// Getting layout positions
    std::string layout;
    if(this->p_nh_.getParam("circlePositions", layout) == true) {
        this->set_circle_positions(this->str2matrix(layout));
        if (this->circlePositions_.size() != this->nclasses_ || this->circlePositions_.at(0).size() != 2){
            ROS_ERROR("The provided layout is not correct. It must be a matrix with %d rows and 2 columns", this->nclasses_);
            return false;
        } 
    }else{
        ROS_ERROR("Parameter 'circlePositions' is mandatory");
        return false;
    }
    // set up the windows layout
	this->setup();

	// Getting thresholds
	if(this->p_nh_.getParam("thresholds", this->thresholds_) == false) {
		ROS_ERROR("Parameter 'thresholds' is mandatory");
		return false;
	} else if(this->thresholds_.size() != this->nclasses_) {
		ROS_ERROR("Thresholds must be same of the number of classes: %d", this->nclasses_);
		return false;
	}	

	// Getting duration parameters
	ros::param::param("~duration/begin", this->duration_.begin, 2000);
	ros::param::param("~duration/boom",  this->duration_.boom, 	1000);
	ros::param::param("~duration/iti", 	 this->duration_.iti,    100);
	ros::param::param("~duration/end", 	 this->duration_.end,   2000);

	return true;
}

void ControlCVSA::run(void) {

	int 	  boomevent;
	int       targethit;
	ros::Rate r(this->rate_);
	this->user_quit_ 	 = false;
	
	// Begin
	ROS_INFO("Protocol started");
	this->sleep(this->duration_.begin);

	while(true) {

		ros::spinOnce();
		this->setevent(Events::CFeedback);
		this->show_center();
		this->has_new_input_ = false;
		this->current_input_ = std::vector<float>(this->nclasses_, 0.5f);
		targethit = -1;
		
		while(ros::ok() && this->user_quit_ == false && targethit == -1) {


			if(this->has_new_input_ == true) {
				this->has_new_input_ = false;
			}

			targethit = this->is_over_threshold(this->current_input_);

			if(targethit != -1)
				break;

			ros::spinOnce();
			r.sleep();
		}
		
		this->hide_center();
		this->setevent(Events::CFeedback + Events::Off);
		if(ros::ok() == false || this->user_quit_ == true) break;

		// boom
		boomevent =  this->classes_.at(targethit);
		this->setevent(boomevent + Events::Command);
		this->show_boom(targethit, targethit);
		this->sleep(this->duration_.boom);
		this->hide_boom();
		this->setevent(boomevent + Events::Command + Events::Off);

		// Inter trial interval
		this->hide_cue();
		this->reset();
		this->sleep(this->duration_.iti);

		if(ros::ok() == false || this->user_quit_ == true) break;
	}

	// End
	ROS_INFO("Protocol ended");
}

std::vector<std::vector<float>> ControlCVSA::str2matrix(const std::string& str) {
	std::vector<std::vector<float>> matrix;
	std::istringstream iss(str);
	std::string row_str;
	while (std::getline(iss, row_str, ';')) {
		std::istringstream row_ss(row_str);
		float value;
		std::vector<float> row_vector;
		while (row_ss >> value) {
			row_vector.push_back(value);
		}
		matrix.push_back(row_vector);
	}

	return matrix;
}

void ControlCVSA::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

	bool class_not_found = false;
	std::vector<int> msgclasses = msg.decoder.classes;

	// Check that the incoming classes are the ones provided
	for(auto it = msgclasses.begin(); it != msgclasses.end(); ++it) {
		auto it2 = std::find(this->classes_.begin(), this->classes_.end(), *it);
		if(it2 == this->classes_.end()) {
			class_not_found = true;
			break;
		}
	}

	if(class_not_found == true) {
		this->has_new_input_ = false;
		ROS_WARN_THROTTLE(5.0f, "The incoming neurooutput message does not have the provided classes");
		return;
	}

	// Set the new incoming data
    this->current_input_ = msg.softpredict.data;
    this->has_new_input_ = true;
}


void ControlCVSA::setevent(int event) {
	this->event_msg_.header.stamp = ros::Time::now();
	this->event_msg_.event = event;
	this->pub_.publish(this->event_msg_);
}

void ControlCVSA::sleep(int msecs) {
	std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

int ControlCVSA::is_over_threshold(std::vector<float>  input) {

	int target = -1;

    for(int i = 0; i < this->nclasses_; i++) {
        if(input.at(i) >= this->thresholds_[i]) {
            target = i;
            break;
        }
    }
    
    return target;
}

} // namespace feedback
