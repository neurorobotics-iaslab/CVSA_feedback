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
    this->arrow_   = new neurodraw::Arrow(0.3f, 0.2f, true, neurodraw::Palette::darkviolet);
    for(int i = 0; i < this->nclasses_; i++) {
        this->circles_.push_back(new neurodraw::Circle(0.15f, true, neurodraw::Palette::white));
    }

    // set the position of the elements
    this->arrow_->rotate(90.0f);
    this->circles_.at(0)->move(-0.85f, -0.75f);
    this->circles_.at(1)->move(0.85f, -0.75f);
    if (this->nclasses_ == 3) {
        this->circles_.at(2)->move(0.0f, 0.85f);
    }

    // add the elements to the engine
    this->engine_->add(this->cross_);
    this->engine_->add(this->arrow_);
    for(int i = 0; i < this->nclasses_; i++) {
        this->engine_->add(this->circles_.at(i));
    }

    this->arrow_->hide();
    this->cross_->hide();
}

void CVSA_layout::reset(void) { 
    for(int i = 0; i < this->nclasses_; i++) {
        this->circles_.at(i)->set_color(neurodraw::Palette::lightgrey);
    }
}

void CVSA_layout::show_fixation(void) {
    this->cross_->show();
}

void CVSA_layout::show_cue(Direction dir) {

    float angle = 0.0;

    switch(dir) {
        case Direction::Leftbottom:
            angle = CueAngle.at(0);
            break;
        case Direction::Rightbottom:
            angle = CueAngle.at(1);
            break;
        case Direction::Up:
            angle = CueAngle.at(2);
            break;
        case Direction::None:
            angle = CueAngle.at(3);
            break;
        default:
            ROS_WARN("Unknown direction required. Cue angle is not set");
            break;
    }

    this->arrow_->rotate(angle);
    this->arrow_->show();
}

void CVSA_layout::show_boom(Direction dir) { // must show the circle in the bottom left or right
    
    neurodraw::Color color = neurodraw::Palette::firebrick;

    switch(dir) {
        case Direction::Leftbottom:
            this->circles_.at(0)->set_color(color);
            break;
        case Direction::Rightbottom:
            this->circles_.at(1)->set_color(color);
            break;
        case Direction::Up:
            this->circles_.at(2)->set_color(color);
            break;
        default:
            break;
    }
}

void CVSA_layout::hide_boom(void) {
    for(int i = 0; i < this->nclasses_; i++) {
        this->circles_.at(i)->set_color(neurodraw::Palette::lightgrey);
    }
}

void CVSA_layout::hide_fixation(void) {
    this->cross_->hide();
}

void CVSA_layout::hide_cue(void) {
    this->arrow_->hide();
}

void CVSA_layout::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

    if(event.state == 0)
        return;

    switch(event.sym) {
        case neurodraw::EventKey::ESCAPE:
               this->engine_->quit();
             this->user_quit_ = true;
                break;
        case neurodraw::EventKey::a:
            this->show_boom(Direction::Leftbottom);
                break;
        case neurodraw::EventKey::d:
            this->show_boom(Direction::Rightbottom);
                break;
        case neurodraw::EventKey::s:
            this->hide_boom();
                break;
        case neurodraw::EventKey::q:
            this->show_cue(Direction::Leftbottom);
                break;
        case neurodraw::EventKey::w:
            this->show_cue(Direction::Up);
                break;
        case neurodraw::EventKey::e:
            this->show_cue(Direction::Rightbottom);
                break;
        case neurodraw::EventKey::r:
            this->hide_cue();
                break;
        case neurodraw::EventKey::f:
            this->show_fixation();
                break;
        case neurodraw::EventKey::g:
            this->hide_fixation();
                break;
    }
}

bool CVSA_layout::set_threshold(float input, Direction dir) {

    switch(dir) {
        case Direction::Leftbottom:
            this->thresholds_[0] = input;
            ROS_WARN("Threshold for Direction::Leftbottom changed to: %f", input);
            break;
        case Direction::Rightbottom:
            this->thresholds_[1] = input;
            ROS_WARN("Threshold for Direction::Rightbottom changed to: %f", input);
            break;
        case Direction::Up:
            this->thresholds_[2] = input;
            ROS_WARN("Threshold for Direction::Up changed to: %f", input);
            break;
        default:
            ROS_ERROR("The provided Direction is unknown. Threshold is not set");
            break;
    }
    
    return true;
}

void CVSA_layout::on_request_reconfigure(config_cvsa &config, uint32_t level) {
    if(std::fabs(config.leftbottom_threshold - this->thresholds_[0]) > 0.00001) {
        this->set_threshold(config.leftbottom_threshold, Direction::Leftbottom);
    }
    if(std::fabs(config.rightbottom_threshold - this->thresholds_[1]) > 0.00001) {
        this->set_threshold(config.rightbottom_threshold, Direction::Rightbottom);
    }
    if(std::fabs(config.up_threshold - this->thresholds_[2]) > 0.00001) {
        this->set_threshold(config.leftbottom_threshold, Direction::Up);
    }
}


 } // namespace feedback

#endif