#ifndef FEEDBACK_CVSA_LAYOUT_CPP_
#define FEEDBACK_CVSA_LAYOUT_CPP_

#include "feedback_cvsa/CVSA_layout.h"

namespace feedback {

CVSA_layout::CVSA_layout(const std::string& wintitle) {
    
    this->engine_ = new neurodraw::Engine(wintitle);
    this->engine_->on_keyboard(&CVSA_layout::on_keyboard_event, this);

    this->user_quit_ = false;

}

CVSA_layout::~CVSA_layout(void) {

    if(this->engine_ != nullptr)
        delete this->engine_;
}


void CVSA_layout::setup(void) {

    // set the default thresholds
    this->thresholds_.resize(this->nclasses_);

    // Bind dynamic reconfigure callback
    this->recfg_callback_type_ = boost::bind(&CVSA_layout::on_request_reconfigure, this, _1, _2);
    this->recfg_srv_.setCallback(this->recfg_callback_type_);

    // create the graphic elements
    this->cross_   = new neurodraw::Cross(0.3f, 0.05f);
    this->center_  = new neurodraw::Circle(0.03f, true, neurodraw::Palette::white);
    this->circle_  = new neurodraw::Circle(0.15f, true, neurodraw::Palette::dimgray);
    this->square_   = new neurodraw::Rectangle(0.2f, 0.2f, true, neurodraw::Palette::white);
    for(int i = 0; i < this->nclasses_; i++) {
        neurodraw::Color color = CuePalette.at(i);
        neurodraw::Ring* ring = new neurodraw::Ring(0.15f,  0.03f, color);
        ring->move(this->circlePositions_(i,0), this->circlePositions_(i,1));
        this->rings_.push_back(ring);
    }

    // add the elements to the engine
    this->engine_->add(this->center_);
    this->engine_->add(this->cross_);
    this->engine_->add(this->square_);
    this->engine_->add(this->circle_);
    for(int i = 0; i < this->nclasses_; i++) {
        this->engine_->add(this->rings_.at(i));
    }

    this->circle_->hide();
    this->square_->hide();
    this->cross_->hide();
    this->center_->hide();
}

void CVSA_layout::reset(void) { 
    this->circle_->hide();
}

void CVSA_layout::show_center(void) {
    this->center_->show();
}

void CVSA_layout::show_fixation(void) {
    this->cross_->show();
}

void CVSA_layout::show_cue(int index) {

    neurodraw::Color color = CuePalette.at(CuePalette.size()-1);

    if(index != -1){
        color = CuePalette.at(index);
    }else{
        ROS_WARN("Unknown direction required. Cue color is not set");
    }

    this->square_->set_color(color);
    this->square_->show();
}

void CVSA_layout::show_boom(int idx_position, int idx_color) { 

    if(idx_position == idx_color){
        this->circle_->move(this->circlePositions_(idx_position,0), this->circlePositions_(idx_position,1));
        this->circle_->set_color(CuePalette.at(idx_color));
    }else if(idx_color == CuePalette.size()-1){
        this->circle_->move(this->circlePositions_(idx_position,0), this->circlePositions_(idx_position,1));
        this->circle_->set_color(CuePalette.at(idx_color));

    }else{
        ROS_WARN("Unknown circle required. Boom color is not set");
    }
    
    this->circle_->show();
}

void CVSA_layout::hide_boom(void) {
    this->circle_->hide();
}

void CVSA_layout::hide_center(void) {
    this->center_->hide();
}

void CVSA_layout::hide_fixation(void) {
    this->cross_->hide();
}

void CVSA_layout::hide_cue(void) {
    this->square_->hide();
}

void CVSA_layout::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

    if(event.state == 0)
        return;

    switch(event.sym) {
        case neurodraw::EventKey::ESCAPE:
               this->engine_->quit();
             this->user_quit_ = true;
                break;
        default:
            break;
    }
}

bool CVSA_layout::set_nclasses(int nclasses){
    this->nclasses_ = nclasses;
    return true;
}
bool CVSA_layout::set_circle_positions(Eigen::MatrixXf circlePositions){
    this->circlePositions_ = circlePositions;
    return true;
}

bool CVSA_layout::set_threshold(float input, int index) {

    if(index != -1){
        this->thresholds_[index] = input;
        ROS_WARN("Threshold for index %d changed to: %f", index, input);
    }else{
        return false;
    }
    
    return true;
}

void CVSA_layout::on_request_reconfigure(config_cvsa &config, uint32_t level) {

    switch (this->nclasses_)
    {
    case 2:
        if(std::fabs(config.threshold_1 - this->thresholds_[1]) > 0.00001) {
            this->set_threshold(config.threshold_1, 1);
        }
        if(std::fabs(config.threshold_2 - this->thresholds_[2]) > 0.00001) {
            this->set_threshold(config.threshold_2, 2);
        }
        break;
    case 3:
        if(std::fabs(config.threshold_1 - this->thresholds_[1]) > 0.00001) {
            this->set_threshold(config.threshold_1, 1);
        }
        if(std::fabs(config.threshold_2 - this->thresholds_[2]) > 0.00001) {
            this->set_threshold(config.threshold_2, 2);
        }
        if(std::fabs(config.threshold_3 - this->thresholds_[3]) > 0.00001) {
            this->set_threshold(config.threshold_3, 3);
        }
        break;
    case 4:
        if(std::fabs(config.threshold_1 - this->thresholds_[1]) > 0.00001) {
            this->set_threshold(config.threshold_1, 1);
        }
        if(std::fabs(config.threshold_2 - this->thresholds_[2]) > 0.00001) {
            this->set_threshold(config.threshold_2, 2);
        }
        if(std::fabs(config.threshold_3 - this->thresholds_[3]) > 0.00001) {
            this->set_threshold(config.threshold_3, 3);
        }
        if(std::fabs(config.threshold_4 - this->thresholds_[4]) > 0.00001) {
            this->set_threshold(config.threshold_4, 4);
        }
        break;
    default:
        break;
    }
}


 } // namespace feedback

#endif