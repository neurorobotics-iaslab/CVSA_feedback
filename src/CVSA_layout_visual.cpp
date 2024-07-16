#include "feedback_cvsa/CVSA_layout_visual.h"

namespace feedback {

CVSA_layout_visual::CVSA_layout_visual(const std::string& wintitle) {
    
    this->engine_ = new neurodraw::Engine(wintitle, 16.f );
    this->engine_->on_keyboard(&CVSA_layout_visual::on_keyboard_event, this);

    this->user_quit_ = false;

}

CVSA_layout_visual::~CVSA_layout_visual(void) {

    if(this->engine_ != nullptr)
        delete this->engine_;
}


void CVSA_layout_visual::setup(void) {

    // create the graphic elements
    this->cross_   = new neurodraw::Cross(0.3f, 0.05f);
    this->center_  = new neurodraw::Circle(0.03f, true, neurodraw::Palette::white);
    this->calibration_  = new neurodraw::Circle(0.03f, true, neurodraw::Palette::white);
    this->circle_  = new neurodraw::Circle(0.15f, true, neurodraw::Palette::dimgray);
    this->square_   = new neurodraw::Rectangle(0.2f, 0.2f, true, neurodraw::Palette::white);
    for(int i = 0; i < this->nclasses_; i++) {
        neurodraw::Color color = CuePalette.at(i);
        neurodraw::Ring* ring = new neurodraw::Ring(0.15f,  0.03f, color);
        ring->move(this->circlePositions_.at(i).at(0), this->circlePositions_.at(i).at(1));
        this->rings_.push_back(ring);

        neurodraw::Circle* circle = new neurodraw::Circle(0.15f, true, color);
        circle->move(this->circlePositions_.at(i).at(0), this->circlePositions_.at(i).at(1));
        this->feedback_circles_.push_back(circle);
    }

    // add the elements to the engine
    this->engine_->add(this->center_);
    this->engine_->add(this->cross_);
    this->engine_->add(this->square_);
    this->engine_->add(this->circle_);
    this->engine_->add(this->calibration_);
    for(int i = 0; i < this->nclasses_; i++) {
        this->engine_->add(this->rings_.at(i));
        this->rings_.at(i)->hide();
    }

    for(int i = 0; i < this->nclasses_; i++) {
        this->engine_->add(this->feedback_circles_.at(i));
        this->feedback_circles_.at(i)->hide();
    }

    this->circle_->hide();
    this->square_->hide();
    this->cross_->hide();
    this->center_->hide();
    this->calibration_->hide();
}

void CVSA_layout_visual::reset(void) { 
    this->circle_->hide();
}

void CVSA_layout_visual::show_center(void) {
    this->center_->show();
}

void CVSA_layout_visual::show_rings_classes(void) {
    for(int i = 0; i < this->nclasses_; i++) {
        this->rings_.at(i)->show();
    }
}

void CVSA_layout_visual::show_fixation(void) {
    this->cross_->show();
}

void CVSA_layout_visual::circle_feedback_update(std::vector<float> radius) {
    for(int i = 0; i < radius.size(); i++) {
        this->feedback_circles_.at(i)->set_radius(radius.at(i));
    }
}

void CVSA_layout_visual::show_circle_feedback(void) {
    for(int i = 0; i < this->nclasses_; i++) {
        this->feedback_circles_.at(i)->show();
    }
}

void CVSA_layout_visual::hide_circle_feedback(void) {
    for(int i = 0; i < this->nclasses_; i++) {
        this->feedback_circles_.at(i)->hide();
    }
}

void CVSA_layout_visual::show_cue(int index) {

    neurodraw::Color color = CuePalette.at(CuePalette.size()-1);

    if(index != -1 && index < CuePalette.size()){
        color = CuePalette.at(index);
    }else{
        ROS_WARN("Unknown direction required. Cue color is not set");
    }

    this->square_->set_color(color);
    this->square_->show();
}

void CVSA_layout_visual::show_boom(int idx_position, int idx_color) { 

    if(idx_position > this->circlePositions_.size() || idx_position < 0){
        ROS_WARN("Unknown circle required. Boom position is not set");
        return;
    }
    if(idx_color >= CuePalette.size() && idx_color < 0){
        ROS_WARN("Unknown color required. Boom position is not set");
        return;
    }
    this->circle_->move(this->circlePositions_.at(idx_color).at(0), this->circlePositions_.at(idx_color).at(1));
    this->circle_->set_color(CuePalette.at(idx_color));
    
    this->circle_->show();
}

void CVSA_layout_visual::show_calibration(std::vector<float> position) {
    this->calibration_->move(position.at(0), position.at(1));
    this->calibration_->show();
}

void CVSA_layout_visual::hide_boom(void) {
    this->circle_->hide();
}

void CVSA_layout_visual::hide_center(void) {
    this->center_->hide();
}

void CVSA_layout_visual::hide_fixation(void) {
    this->cross_->hide();
}

void CVSA_layout_visual::hide_cue(void) {
    this->square_->hide();
}

void CVSA_layout_visual::hide_calibration(void) {
    this->calibration_->hide();
}

void CVSA_layout_visual::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

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

bool CVSA_layout_visual::set_nclasses(int nclasses){
    this->nclasses_ = nclasses;
    return true;
}

bool CVSA_layout_visual::set_circle_positions(std::vector<std::vector<float>> circlePositions){
    this->circlePositions_ = circlePositions;
    return true;
}

} // namespace feedback