// TODO: Upgrade for three classes
#include "feedback_cvsa/TrainingArrow.h"


namespace feedback {

TrainingArrow::TrainingArrow(void) : SingleArrow("trainingarrow", 2), p_nh_("~") {


	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
	this->sub_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &TrainingArrow::on_received_data, this);

}

TrainingArrow::~TrainingArrow(void) {}

bool TrainingArrow::configure(void) {

	int mindur_active, maxdur_active, mindur_rest, maxdur_rest;
	std::vector<float> thresholds;
	std::string modality;

	// Getting classes
	if(this->p_nh_.getParam("classes", this->classes_) == false) {
		ROS_ERROR("Parameter 'classes' is mandatory");
		return false;
	} else if(this->classes_.size() != 2 && this->classes_.size() != 3) {
		ROS_ERROR("The provided number of classes must be 2 o 3, now it is: %ld", this->classes_.size());
		return false;
	}
	this->nclasses_ = this->classes_.size();

	// Getting thresholds
	if(this->p_nh_.getParam("thresholds", thresholds) == false) {
		ROS_ERROR("Parameter 'thresholds' is mandatory");
		return false;
	} else if(thresholds.size() != this->nclasses_) {
		ROS_ERROR("Thresholds must be the same of the number of classes");
		return false;
	}

	// Getting trials per class
	if(this->p_nh_.getParam("trials", this->trials_per_class_) == false) {
		ROS_ERROR("Parameter 'trials' is mandatory");
		return false;
	} else if(this->trials_per_class_.size() != this->nclasses_) { 
		ROS_ERROR("Number of trials per class must be provided for each class");
		return false;
	}
	
	// Getting modality 
	if(this->p_nh_.getParam("modality", modality) == false) {
		ROS_ERROR("Parameter 'modality' is mandatory");
		return false;
	}
	
	if(modality.compare("calibration") == 0) {
		this->modality_ = Modality::Calibration;
	} else if(modality.compare("evaluation") == 0) {
		this->modality_ = Modality::Evaluation;
	} else {
		ROS_ERROR("Unknown modality provided");
		return false;
	}

	// Getting show on rest
	ros::param::param("~show_on_rest", this->show_on_rest_, true);

	// Getting duration parameters
	ros::param::param("~duration/begin", 		   this->duration_.begin, 		    5000);
	ros::param::param("~duration/start", 		   this->duration_.start, 		    1000);
	ros::param::param("~duration/fixation", 	   this->duration_.fixation, 	    2000);
	ros::param::param("~duration/cue", 			   this->duration_.cue, 		    1000);
	ros::param::param("~duration/feedback_min",    this->duration_.feedback_min,    4000);
	ros::param::param("~duration/feedback_max",    this->duration_.feedback_max,    5500);
	ros::param::param("~duration/boom", 		   this->duration_.boom, 		    1000);
	ros::param::param("~duration/timeout", 		   this->duration_.timeout, 	   10000);
	ros::param::param("~duration/timeout_on_rest", this->duration_.timeout_on_rest, 6000);
	ros::param::param("~duration/iti", 			   this->duration_.iti, 		   	 100);
	ros::param::param("~duration/end", 			   this->duration_.end, 		  	2000);


	// Setting parameters
	if(this->modality_ == Modality::Calibration) {
		mindur_active = this->duration_.feedback_min;
		maxdur_active = this->duration_.feedback_max;
		mindur_rest   = this->duration_.feedback_min;
        maxdur_rest   = this->duration_.feedback_max;
	} else {
		mindur_active = this->duration_.timeout;
		maxdur_active = this->duration_.timeout;
		mindur_rest   = this->duration_.timeout_on_rest;
        maxdur_rest   = this->duration_.timeout_on_rest;
	}

	this->trialsequence_.addclass(this->classes_.at(0), this->trials_per_class_.at(0), mindur_active, maxdur_active);
	this->trialsequence_.addclass(this->classes_.at(1), this->trials_per_class_.at(1), mindur_active, maxdur_active);
	if(this->classes_.size() == 3) 
		this->trialsequence_.addclass(this->classes_.at(2), this->trials_per_class_.at(2), mindur_rest, maxdur_rest);

	this->set_threshold(thresholds.at(0), Direction::Leftbottom);
	this->set_threshold(thresholds.at(1), Direction::Rightbottom);
	if(this->nclasses_ == 3)
		this->set_threshold(thresholds.at(2), Direction::Up);
	
	ROS_INFO("Total number of classes: %ld", this->classes_.size());
	ROS_INFO("Total number of trials:  %d", this->trialsequence_.size());
	ROS_INFO("Trials have been randomized");

	return true;

}

TrainingArrow::Direction TrainingArrow::class2direction(int eventcue) {
	Direction dir = Direction::None;

	auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
	
	if(it != this->classes_.end())
		dir = static_cast<Direction>(it - this->classes_.begin());

	return dir;
}

void TrainingArrow::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

	int refclass = this->classes_.at(0);
	int refclassIdx;
	bool class_not_found = false;
	std::vector<int> msgclasses = msg.decoder.classes;

	// First: check that the incoming classes are the ones provided
	for(auto it = msgclasses.begin(); it != msgclasses.end(); ++it) {
		auto it2 = std::find(this->classes_.begin(), this->classes_.end(), *it);
		if(it2 == this->classes_.end()) {
			class_not_found = true;
			break;
		}
	}

	// Second: find the index of the refclass
	if(class_not_found == true) {
		this->has_new_input_ = false;
		ROS_WARN_THROTTLE(5.0f, "The incoming neurooutput message does not have the provided classes");
		return;
	}

	auto it = std::find(msgclasses.begin(), msgclasses.end(), refclass);

	if(it != msgclasses.end()) {
		refclassIdx = it - msgclasses.begin();

		this->current_input_ = msg.softpredict.data.at(refclassIdx);
		//this->current_angle_ = this->input2angle(msg.softpredict.data.at(refclassIdx)); // nothing to increase in CVSA
		this->has_new_input_ = true;
	} else {
		this->has_new_input_ = true;
		ROS_WARN_THROTTLE(5.0f, "Cannot find class %d in the incoming message", refclass);
	}
		
}


void TrainingArrow::run(void) {
/*
	int 	  trialnumber;
	int 	  trialclass;
	int 	  trialduration;
	float	  trialthreshold;
	int		  hitclass;
	int 	  boomevent;
	Direction trialdirection;
	Direction targethit;
	ros::Rate r(this->rate_);

	LinearPilot linearpilot(1000.0f/this->rate_);
	SinePilot   sinepilot(1000.0f/this->rate_, 0.25f, 0.5f);
	Autopilot*  autopilot;

	ROS_INFO("Protocol started");
	
	// Begin
	this->sleep(this->duration_.begin);
	
	for(auto it = this->trialsequence_.begin(); it != this->trialsequence_.end(); ++it) {
		
		// Getting trial information
		trialnumber    = (it - this->trialsequence_.begin()) + 1;
		trialclass     = (*it).classid;
		trialduration  = (*it).duration;
		trialdirection = this->class2direction(trialclass);
		trialthreshold = trialdirection == Direction::Left ? thresholds_.at(0) : thresholds_.at(1);
		targethit      = Direction::None;

		if(this->modality_ == Modality::Calibration) {
			autopilot = trialdirection == Direction::Forward ? (Autopilot*)(&sinepilot) : (Autopilot*)(&linearpilot);
			autopilot->set(0.5f, trialthreshold, trialduration); 
		}

		ROS_INFO("Trial %d/%d (class: %d | duration: %d ms)", trialnumber, this->trialsequence_.size(), trialclass, trialduration);
		this->setevent(Events::Start);
		this->sleep(this->duration_.start);
		this->setevent(Events::Start + Events::Off);

		if(ros::ok() == false || this->user_quit_ == true) break;
		
		// Fixation
		this->setevent(Events::Fixation);
		this->show_fixation();
		this->sleep(this->duration_.fixation);
		this->hide_fixation();
		this->setevent(Events::Fixation + Events::Off);

		if(ros::ok() == false || this->user_quit_ == true) break;

		// Cue
		this->setevent(trialclass);
		this->show_cue(trialdirection);
		this->sleep(this->duration_.cue);
		this->setevent(trialclass + Events::Off);
		
		if(ros::ok() == false || this->user_quit_ == true) break;

		// Continuous Feedback
		this->timer_.tic();

		// Consuming old messages
		ros::spinOnce();

		// Send reset event
		this->setevent(Events::CFeedback);
		this->has_new_input_ = false;
		this->current_input_ = 0.5f;
		
		while(ros::ok() && this->user_quit_ == false && targethit == Direction::None) {

			if(this->modality_ == Modality::Calibration) {
				if(trialdirection == Direction::Forward && this->show_on_rest_ == false) {
					this->current_input_ = this->current_input_;
				} else {
					this->current_input_ = this->current_input_ + autopilot->step();
				}
				this->move(this->input2angle(this->current_input_));
			} else if(this->modality_ == Modality::Evaluation) {
				if(this->has_new_input_ == true) {
					this->move(this->input2angle(this->current_input_));
					this->has_new_input_ = false;
				}
			}
			

			targethit = this->is_target_hit(this->current_input_, trialdirection, 
											this->timer_.toc(), trialduration);

			if(targethit != Direction::None)
				break;
		
			r.sleep();
			ros::spinOnce();
		}
		this->setevent(Events::CFeedback + Events::Off);
		if(ros::ok() == false || this->user_quit_ == true) break;
		

		// Boom
		boomevent = trialdirection == targethit ? Events::Hit : Events::Miss;
		this->setevent(boomevent);
		this->show_boom(targethit);
		this->sleep(this->duration_.boom);
		this->hide_boom();
		this->setevent(boomevent + Events::Off);

		//this->setevent(Events::Start + Events::Off);

		switch(boomevent) {
			case Events::Hit:
				ROS_INFO("Target hit");
				break;
			case Events::Miss:
				ROS_INFO("Target miss");
				break;
		}

		if(ros::ok() == false || this->user_quit_ == true) break;

		// Inter trial interval
		this->hide_cue();
		this->reset();
		this->sleep(this->duration_.iti);

		if(ros::ok() == false || this->user_quit_ == true) break;

	}

	// End
	if(user_quit_ == false)
		this->sleep(this->duration_.end);
	ROS_INFO("Protocol ended");

*/

}

void TrainingArrow::setevent(int event) {

	this->event_msg_.header.stamp = ros::Time::now();
	this->event_msg_.event = event;
	this->pub_.publish(this->event_msg_);
	
}

void TrainingArrow::sleep(int msecs) {
	std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

TrainingArrow::Direction TrainingArrow::is_target_hit(float input, Direction direction, int elapsed, int duration) {

	Direction target = Direction::None;

	if(input >= thresholds_[0]) {
		target = Direction::Leftbottom;
	} else if(input <= thresholds_[1]) {
		target = Direction::Rightbottom;
	} else if(direction == Direction::Up && elapsed >= duration) {
		target = Direction::Up;
	} else if(elapsed >= duration) {
		target = Direction::Timeout;
	}
	
	return target;
}

} // namespace feedback

