#include "feedback_cvsa/TrainingCVSA_visual.h"


namespace feedback {

TrainingCVSA_visual::TrainingCVSA_visual(void) : CVSA_layout_visual("trainingCVSA_visual"), p_nh_("~") {

    this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("events/bus", 1);
    this->sub_ = this->nh_.subscribe("cvsa/neuroprediction/integrated", 1, &TrainingCVSA_visual::on_received_data, this);
    this->srv_camera_ready_ = this->nh_.serviceClient<std_srvs::Trigger>("cvsa/camera_ready");
}

TrainingCVSA_visual::~TrainingCVSA_visual(void) {}

bool TrainingCVSA_visual::configure(void) {

    /* Bind dynamic reconfigure callback */
    this->recfg_callback_type_ = boost::bind(&TrainingCVSA_visual::on_request_reconfigure, this, _1, _2);
    this->recfg_srv_.setCallback(this->recfg_callback_type_);

    std::string modality;

    /* PARAMETERS FOR THE LAYOUT */
    // Getting classes
    if(this->p_nh_.getParam("classes", this->classes_) == false) {
        ROS_ERROR("Parameter 'classes' is mandatory");
        return false;
    } 
    this->set_nclasses(this->classes_.size());

    // Getting layout positions
    std::string layout;
    if(this->p_nh_.getParam("circlePositions", layout) == true) {
        if (this->str2matrix(layout).size() != this->nclasses_ || this->str2matrix(layout).at(0).size() != 2){
            ROS_ERROR("The provided layout is not correct. It must be a matrix with %d rows and 2 columns", this->nclasses_);
            return false;
        } 
        this->set_circle_positions(this->str2matrix(layout));
    }else{
        ROS_ERROR("Parameter 'circlePositions' is mandatory");
        return false;
    }

    // set up the windows layout for the feedback
	this->setup();


    /* PARAMETER FOR THE TRIAL EXECUTIONS*/
    // Getting thresholds
    if(this->p_nh_.getParam("thresholds", this->thresholds_) == false) {
        ROS_ERROR("Parameter 'thresholds' is mandatory");
        return false;
    } else if(this->thresholds_.size() != this->nclasses_) {
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

    /* PARAMETER FOR THE EYE*/
    // Getting do or not eye calibration
    if(this->p_nh_.getParam("eye_calibration", this->eye_calibration_) == false) {
        ROS_ERROR("Parameter 'eye_calibration' is mandatory");
        return false;
    } 
    if(this->eye_calibration_ == true){
        if(this->p_nh_.getParam("calibration_classes", this->calibration_classes_) == false) {
            ROS_ERROR("Parameter 'calibration_classes' is mandatory since eye_calibration is true");
            return false;
        } 
        if(this->p_nh_.getParam("calibration_positions", layout) == true) {
            if (this->str2matrix(layout).size() != this->calibration_classes_.size() || this->str2matrix(layout).at(0).size() != 2){
                ROS_ERROR("The provided layout for calibration_positions is not correct. It must be a matrix with %ld rows and 2 columns", this->calibration_classes_.size());
                return false;
            } 
            this->calibration_positions_ = this->str2matrix(layout);
        }else{
            ROS_ERROR("Parameter 'calibration_positions' is mandatory since eye_calibration is true");
            return false;
        }
        if(this->p_nh_.getParam("max_trials_per_class", this->max_trials_per_class_) == false) {
            ROS_ERROR("Parameter 'max_trials_per_class' is mandatory since eye_calibration is true");
            if(this->max_trials_per_class_.size() < this->nclasses_){
                ROS_ERROR("The number of max_trials_per_class must be greater than the number of classes");
            }
            return false;
        }else{
            int sum;
            for(int i = 0; i < this->trials_per_class_.size(); i++){
                if(this->max_trials_per_class_.at(i) < this->trials_per_class_.at(i)){
                    ROS_ERROR("The number of max_trials_per_class_ must be greater than the trials_per_class_ for each class of the trials per class");
                    return false;
                }
            }
        }
        this->srv_repeat_trial_ = this->nh_.advertiseService("cvsa/repeat_trial", &TrainingCVSA_visual::on_repeat_trial, this);
        this->pub_trials_keep_ = this->nh_.advertise<feedback_cvsa::Trials_to_keep>("cvsa/trials_keep", 1);
    }

    // Getting duration parameters
    ros::param::param("~duration/begin",            this->duration_.begin,             5000);
    ros::param::param("~duration/start",            this->duration_.start,             1000);
    ros::param::param("~duration/fixation",         this->duration_.fixation,          2000);
    ros::param::param("~duration/cue",              this->duration_.cue,               1000);
    ros::param::param("~duration/feedback_min",     this->duration_.feedback_min,      4000);
    ros::param::param("~duration/feedback_max",     this->duration_.feedback_max,      5500);
    ros::param::param("~duration/boom",             this->duration_.boom,              1000);
    ros::param::param("~duration/timeout",          this->duration_.timeout,           5000);
    ros::param::param("~duration/iti",              this->duration_.iti,                100);
    ros::param::param("~duration/end",              this->duration_.end,               2000);
    ros::param::param("~duration/calibration",      this->duration_.calibration,       2000);


    // Setting parameters
    if(this->modality_ == Modality::Calibration) {
        this->mindur_active_ = this->duration_.feedback_min;
        this->maxdur_active_ = this->duration_.feedback_max;
    } else {
        this->mindur_active_ = this->duration_.timeout;
        this->maxdur_active_ = this->duration_.timeout;
    }

    for(int i = 0; i < this->nclasses_; i++) {
        this->trialsequence_.addclass(this->classes_.at(i), this->trials_per_class_.at(i), this->mindur_active_, this->maxdur_active_);
    }
    
    ROS_INFO("Total number of classes: %ld", this->classes_.size());
    ROS_INFO("Total number of trials:  %d", this->trialsequence_.size());
    ROS_INFO("Trials have been randomized");

    return true;

}

int TrainingCVSA_visual::class2direction(int eventcue) {

    auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
    
    if(it != this->classes_.end())
        return int(it - this->classes_.begin());

    return -1;
}

int TrainingCVSA_visual::class2index(int eventcue) {

    auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
    int idx = -1;

    if(it != this->classes_.end()){
        idx = std::distance(this->classes_.begin(), it);
    }else{
        ROS_ERROR("Class %d not found", eventcue);
    }

    return idx;
}

float TrainingCVSA_visual::direction2threshold(int index) {

	if(index != -1) {
		return this->thresholds_[index];
	} else {
		ROS_ERROR("Unknown direction");
		return -1;
	}
}

std::vector<std::vector<float>> TrainingCVSA_visual::str2matrix(const std::string& str) {
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

void TrainingCVSA_visual::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

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
        ROS_WARN_THROTTLE(5.0f, "The incoming neurooutput message does not have the provided classes");
        return;
    }

    // Set the new incoming data
    this->current_input_ = msg.softpredict.data;
        
}

bool TrainingCVSA_visual::on_repeat_trial(feedback_cvsa::Repeat_trial::Request &req, feedback_cvsa::Repeat_trial::Response &res) {
    this->trial_ok_ = 0;
    int class2repeat = req.class2repeat;
    auto it = std::find(this->classes_.begin(), this->classes_.end(), class2repeat);
    if(it != this->classes_.end()) {
        int idx_class = std::distance(this->classes_.begin(), it);
        if(this->trials_per_class_.at(idx_class) <= this->max_trials_per_class_.at(idx_class)){
            this->trials_per_class_.at(idx_class) = this->trials_per_class_.at(idx_class) + 1;
            this->trialsequence_.addtrial(class2repeat, this->mindur_active_, this->maxdur_active_);
            res.success = true;
            return true;
        }
        
    }
    
    res.success = false;
    return false;
    
}


void TrainingCVSA_visual::run(void) {

    this->srv_camera_ready_.waitForExistence();
    std_srvs::Trigger srv = std_srvs::Trigger();

    while(this->srv_camera_ready_.call(srv.request, srv.response)){
        if(srv.response.success == false) {
            ROS_WARN_ONCE("Camera is not ready");
        }else{
            break;
        }
    }
    
    ROS_INFO("Camera is ready");
    if(this->eye_calibration_){
        ROS_INFO("Calibration eye started");
        this->eye_calibration();
    }

    ROS_INFO("Protocol BCI started");
    // show the rings and the center of the feedback of each class in the drawing window
    this->show_rings_classes();
    this->hide_circle_feedback();
    std::vector<float> radius = this->input2radius(std::vector<float>(this->nclasses_, 0.0f));
    this->circle_feedback_update(radius);
    this->show_circle_feedback();
    this->bci_protocol();

}

void TrainingCVSA_visual::eye_calibration(void) {

    this->sleep(this->duration_.begin);
    this->show_fixation();
    this->sleep(this->duration_.fixation);
    this->hide_fixation();

    // randomize the order of the calibration classes
    std::vector<int> idx_class;
    for(int i = 0; i < this->calibration_classes_.size(); i++) 
        idx_class.push_back(i);

    std::random_device rnddev;
    std::mt19937 rndgen(rnddev());

    std::shuffle(std::begin(idx_class), std::end(idx_class), rndgen);

    // Start the calibration
    for(int i = 0; i < this->calibration_classes_.size(); i++) {
        this->setevent(Events::StartCalibEye);
        //std::cout << calibration_classes_.at(idx_class.at(i)) << std::endl;
        this->sleep(this->duration_.iti);
        this->setevent(calibration_classes_.at(idx_class.at(i)));
        this->show_calibration(this->calibration_positions_.at(idx_class.at(i)));
        this->sleep(this->duration_.calibration);
        this->hide_calibration();
        this->setevent(calibration_classes_.at(idx_class.at(i)) + Events::Off);
        this->sleep(this->duration_.iti);
        this->setevent(Events::StartCalibEye + Events::Off);
        this->sleep(this->duration_.iti);
    }
    
}

void TrainingCVSA_visual::bci_protocol(void){
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
    std::vector<int> idxs_classes(this->nclasses_);
    std::iota(idxs_classes.begin(), idxs_classes.end(), 0);

    rosneuro::feedback::LinearPilot linearpilot(1000.0f/this->rate_);
    rosneuro::feedback::Autopilot*  autopilot;

    // Begin
    this->sleep(this->duration_.begin);
    
    for(int i = 0; i < this->trialsequence_.size(); i++) {
        // Getting trial information
        trialnumber    = i + 1;
        Trial t = this->trialsequence_.gettrial(i);
        trialclass     = t.classid;
        trialduration  = t.duration;
        idx_class      = this->class2index(trialclass); 
        trialdirection = this->class2direction(trialclass);
        trialthreshold = this->direction2threshold(trialdirection);
        targethit      = -1;
        this->trial_ok_ = 1;
        std::vector<int> idxs_classes_not_trial = idxs_classes;
        auto tmp_vector = std::remove(idxs_classes_not_trial.begin(), idxs_classes_not_trial.end(), idx_class);
        idxs_classes_not_trial.erase(tmp_vector, idxs_classes_not_trial.end());

        if(this->modality_ == Modality::Calibration) {
            autopilot = &linearpilot;
            autopilot->set(0.0f, trialthreshold, trialduration); 
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

        // Set up initial probabilities
        this->current_input_ = std::vector<float>(this->nclasses_, 0.0f);

        while(ros::ok() && this->user_quit_ == false && targethit == -1) {

            if(this->modality_ == Modality::Calibration) {
                this->hide_circle_feedback();
                std::vector<float> radius = this->input2radius(this->current_input_);
                this->circle_feedback_update(radius);
                this->show_circle_feedback();
                this->current_input_[idx_class] = this->current_input_[idx_class] + autopilot->step();
                for(auto it = idxs_classes_not_trial.begin(); it != idxs_classes_not_trial.end(); ++it) {
                    //this->current_input_[*it] = (1.0f - this->current_input_[idx_class]) / (this->nclasses_ - 1);
                    this->current_input_[*it] = 0.0f;
                }
                //ROS_INFO("Probabilities: %f %f Thresholds: %f %f", this->current_input_[0], this->current_input_[1], this->thresholds_[0], this->thresholds_[1]);
            } else if(this->modality_ == Modality::Evaluation) {
                this->hide_circle_feedback();
                std::vector<float> radius = this->input2radius(this->current_input_);
                this->circle_feedback_update(radius);
                this->show_circle_feedback();
                //ROS_INFO("Probabilities: %f %f Thresholds: %f %f", this->current_input_[0], this->current_input_[1], this->thresholds_[0], this->thresholds_[1]);
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

        // Set back the visual feedback
        this->hide_circle_feedback();
        std::vector<float> radius = this->input2radius(std::vector<float>(this->nclasses_, 0.0f));
        this->circle_feedback_update(radius);
        this->show_circle_feedback();

        if(ros::ok() == false || this->user_quit_ == true) break;
        this->trials_keep_.push_back(this->trial_ok_);

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

    // Publish the trials keep
    feedback_cvsa::Trials_to_keep msg;
    msg.trials_to_keep = this->trials_keep_;
    this->pub_trials_keep_.publish(msg);
}

std::vector<float> TrainingCVSA_visual::normalize(std::vector<float>& input) {
    std::vector<float> input_norm(input.size());
    for(int i = 0; i < this->nclasses_; i++){
        input_norm.at(i) = input.at(i) - 1.0f/this->nclasses_;
        float ths = this->thresholds_.at(i) - 1.0f/this->nclasses_;
        if(input_norm.at(i) < 0)
            input_norm.at(i) = 0;
        input_norm.at(i) = input_norm.at(i) / ths;
    }

    return input_norm;
}

void TrainingCVSA_visual::setevent(int event) {

    this->event_msg_.header.stamp = ros::Time::now();
    this->event_msg_.event = event;
    this->pub_.publish(this->event_msg_);
}

void TrainingCVSA_visual::sleep(int msecs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

int TrainingCVSA_visual::is_target_hit(std::vector<float> input, int elapsed, int duration) {

    int target = -1;
    double epsilon = 0.00001;

    for(int i = 0; i < this->nclasses_; i++) {
        if(input.at(i) >= this->thresholds_[i] - epsilon) {
            target = i;
            break;
        } else if(elapsed >= duration) {
            target = CuePalette.size()-1;
            ROS_INFO("Timeout");
        }
    }
    
    return target;
}

std::vector<float> TrainingCVSA_visual::input2radius(std::vector<float> inputs) {
    int n = inputs.size();
    std::vector<float> radius(n, 0.0f);
    for(int i = 0; i < n; i++){
        float c_input = inputs.at(i);
        if(c_input >= 0.0f && c_input <= 1.0f) {
            radius.at(i) = c_input * (0.15f - 0.03f); // radius ring - tickness ring
        } else {
            ROS_WARN("Input %d value out of range [0,1]", i);
        }
    }
    
	return radius;
}


void TrainingCVSA_visual::on_request_reconfigure(config_cvsa &config, uint32_t level) {

    switch (this->nclasses_)
    {
    case 2:
        if(std::fabs(config.threshold_0 - this->thresholds_[0]) > 0.00001) {
            this->thresholds_[0] = config.threshold_0;
        }
        if(std::fabs(config.threshold_1 - this->thresholds_[1]) > 0.00001) {
            this->thresholds_[1] = config.threshold_1;
        }
        break;
    case 3:
        if(std::fabs(config.threshold_0 - this->thresholds_[0]) > 0.00001) {
            this->thresholds_[0] = config.threshold_0;
        }
        if(std::fabs(config.threshold_1 - this->thresholds_[1]) > 0.00001) {
            this->thresholds_[1] = config.threshold_1;
        }
        if(std::fabs(config.threshold_2 - this->thresholds_[2]) > 0.00001) {
            this->thresholds_[2] = config.threshold_2;
        }
        break;
    case 4:
        if(std::fabs(config.threshold_0 - this->thresholds_[0]) > 0.00001) {
            this->thresholds_[0] = config.threshold_0;
        }
        if(std::fabs(config.threshold_1 - this->thresholds_[1]) > 0.00001) {
            this->thresholds_[1] = config.threshold_1;
        }
        if(std::fabs(config.threshold_2 - this->thresholds_[2]) > 0.00001) {
            this->thresholds_[2] = config.threshold_2;
        }
        if(std::fabs(config.threshold_3 - this->thresholds_[3]) > 0.00001) {
            this->thresholds_[3] = config.threshold_3;
        }
        break;
    default:
        break;
    }
}

} // namespace feedback