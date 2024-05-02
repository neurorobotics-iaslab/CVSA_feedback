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

    // create the graphic elements
    this->cross_   = new neurodraw::Cross(0.3f, 0.05f);
    this->center_  = new neurodraw::Circle(0.03f, true, neurodraw::Palette::white);
    this->circle_  = new neurodraw::Circle(0.15f, true, neurodraw::Palette::dimgray);
    this->square_   = new neurodraw::Rectangle(0.2f, 0.2f, true, neurodraw::Palette::white);
    for(int i = 0; i < this->nclasses_; i++) {
        neurodraw::Color color = CuePalette.at(i);
        neurodraw::Ring* ring = new neurodraw::Ring(0.15f,  0.03f, color);
        ring->move(this->circlePositions_.at(i).at(0), this->circlePositions_.at(i).at(1));
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

    if(index != -1 && index < CuePalette.size()){
        color = CuePalette.at(index);
    }else{
        ROS_WARN("Unknown direction required. Cue color is not set");
    }

    this->square_->set_color(color);
    this->square_->show();
}

void CVSA_layout::show_boom(int idx_position, int idx_color) { 

    if(idx_position > this->circlePositions_.size() || idx_position < 0){
        ROS_WARN("Unknown circle required. Boom position is not set");
        return;
    }
    if(idx_color >= CuePalette.size() && idx_color < 0){
        ROS_WARN("Unknown color required. Boom position is not set");
        return;
    }
    this->circle_->move(this->circlePositions_.at(idx_position).at(0), this->circlePositions_.at(idx_position).at(1));
    this->circle_->set_color(CuePalette.at(idx_color));
    
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

bool CVSA_layout::set_circle_positions(std::vector<std::vector<float>> circlePositions){
    this->circlePositions_ = circlePositions;
    return true;
}

} // namespace feedback