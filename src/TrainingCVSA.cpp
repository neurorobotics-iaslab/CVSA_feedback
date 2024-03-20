#include "feedback_cvsa/TrainingCVSA.h"


namespace feedback {

TrainingCVSA::TrainingCVSA(void) : CVSA_layout("trainingCVSA"), p_nh_("~") {

    this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
    this->sub_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &TrainingCVSA::on_received_data, this);

}

TrainingCVSA::~TrainingCVSA(void) {}

bool TrainingCVSA::configure(void) {

    int mindur_active, maxdur_active, mindur_rest, maxdur_rest;
    std::vector<float> thresholds;
    std::string modality;

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
    if(this->p_nh_.getParam("thresholds", thresholds) == false) {
        ROS_ERROR("Parameter 'thresholds' is mandatory");
        return false;
    } else if(thresholds.size() != this->nclasses_) {
        ROS_ERROR("Thresholds must be the same of the number of classes %d", this->nclasses_);
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

    // Getting duration parameters
    ros::param::param("~duration/begin",            this->duration_.begin,             5000);
    ros::param::param("~duration/start",            this->duration_.start,             1000);
    ros::param::param("~duration/fixation",         this->duration_.fixation,          2000);
    ros::param::param("~duration/cue",              this->duration_.cue,               1000);
    ros::param::param("~duration/feedback_min",     this->duration_.feedback_min,      4000);
    ros::param::param("~duration/feedback_max",     this->duration_.feedback_max,      5500);
    ros::param::param("~duration/boom",             this->duration_.boom,              1000);
    ros::param::param("~duration/timeout",          this->duration_.timeout,          10000);
    ros::param::param("~duration/iti",              this->duration_.iti,                100);
    ros::param::param("~duration/end",              this->duration_.end,               2000);


    // Setting parameters
    if(this->modality_ == Modality::Calibration) {
        mindur_active = this->duration_.feedback_min;
        maxdur_active = this->duration_.feedback_max;
    } else {
        mindur_active = this->duration_.timeout;
        maxdur_active = this->duration_.timeout;
    }

    for(int i = 0; i < this->nclasses_; i++) {
        this->trialsequence_.addclass(this->classes_.at(i), this->trials_per_class_.at(i), mindur_active, maxdur_active);
    }

    for(int i = 0; i < this->nclasses_; i++) {
        this->set_threshold(thresholds.at(i), i);
    }
    
    ROS_INFO("Total number of classes: %ld", this->classes_.size());
    ROS_INFO("Total number of trials:  %d", this->trialsequence_.size());
    ROS_INFO("Trials have been randomized");

    return true;

}

int TrainingCVSA::class2direction(int eventcue) {

    auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
    
    if(it != this->classes_.end())
        return int(it - this->classes_.begin());

    return -1;
}

int TrainingCVSA::class2index(int eventcue) {

    auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
    int idx = -1;

    if(it != this->classes_.end()){
        idx = std::distance(this->classes_.begin(), it);
    }else{
        ROS_ERROR("Class %d not found", eventcue);
    }

    return idx;
}

float TrainingCVSA::direction2threshold(int index) {

	if(index != -1) {
		return this->thresholds_[index];
	} else {
		ROS_ERROR("Unknown direction");
		return -1;
	}
}

std::vector<std::vector<float>> TrainingCVSA::str2matrix(const std::string& str) {
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

void TrainingCVSA::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

    // Check if the incoming message has the provided classes
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


void TrainingCVSA::run(void) {

    int       trialnumber;
    int       trialclass;
    int       trialduration;
    float     trialthreshold;
    int       hitclass;
    int       boomevent;
    int       idx_class;
    int       trialdirection;
    int       targethit;
    ros::Rate r(this->rate_);

    rosneuro::feedback::LinearPilot linearpilot(1000.0f/this->rate_);
    rosneuro::feedback::Autopilot*  autopilot;

    ROS_INFO("Protocol started");
    
    // Begin
    this->sleep(this->duration_.begin);
    
    for(auto it = this->trialsequence_.begin(); it != this->trialsequence_.end(); ++it) {
        
        // Getting trial information
        trialnumber    = (it - this->trialsequence_.begin()) + 1;
        trialclass     = (*it).classid;
        trialduration  = (*it).duration;
        idx_class      = this->class2index(trialclass); 
        trialdirection = this->class2direction(trialclass);
        trialthreshold = this->direction2threshold(trialdirection);
        targethit      = -1;

        if(this->modality_ == Modality::Calibration) {
            autopilot = &linearpilot;
            autopilot->set(0.5f, trialthreshold, trialduration); 
        }

        ROS_INFO("Trial %d/%d (class: %d | duration: %d ms)", trialnumber, this->trialsequence_.size(), trialclass, trialduration);
        this->setevent(Events::Start);
        this->sleep(this->duration_.start);
        //this->setevent(Events::Start + Events::Off);

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
        this->hide_cue();
        this->setevent(trialclass + Events::Off);
        
        if(ros::ok() == false || this->user_quit_ == true) break;

        // Continuous Feedback
        this->timer_.tic();

        // Consuming old messages
        ros::spinOnce();

        // Send reset event
        this->setevent(Events::CFeedback);
        this->show_center();
        this->has_new_input_ = false;

        this->current_input_ = std::vector<float>(this->nclasses_, 0.5f);
        
        while(ros::ok() && this->user_quit_ == false && targethit == -1) {

            if(this->modality_ == Modality::Calibration) {
                this->current_input_[idx_class] = this->current_input_[idx_class] + autopilot->step();
            } else if(this->modality_ == Modality::Evaluation) {
                if(this->has_new_input_ == true) {
                    this->has_new_input_ = false;
                }
            }
            
            targethit = this->is_target_hit(this->current_input_,  
                                            this->timer_.toc(), trialduration);

            if(targethit != -1)
                break;
        
            r.sleep();
            ros::spinOnce();
        }
        this->hide_center();
        this->setevent(Events::CFeedback + Events::Off);
        if(ros::ok() == false || this->user_quit_ == true) break;
        

        // Boom
        boomevent = trialdirection == targethit ? Events::Hit : Events::Miss;
        this->setevent(boomevent);
        this->show_boom(trialdirection, targethit);
        this->sleep(this->duration_.boom);
        this->hide_boom();
        this->setevent(boomevent + Events::Off);

        // Finish the trial
        this->setevent(Events::Start + Events::Off);

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

}

void TrainingCVSA::setevent(int event) {

    this->event_msg_.header.stamp = ros::Time::now();
    this->event_msg_.event = event;
    this->pub_.publish(this->event_msg_);
}

void TrainingCVSA::sleep(int msecs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

int TrainingCVSA::is_target_hit(std::vector<float> input, int elapsed, int duration) {

    int target = -1;

    for(int i = 0; i < this->nclasses_; i++) {
        if(elapsed >= duration) {
            target = CuePalette.size()-1;
            ROS_INFO("Timeout");
        }else if(input.at(i) >= this->thresholds_[i]) {
            target = i;
            break;
        }
    }
    
    return target;
}

} // namespace feedback

