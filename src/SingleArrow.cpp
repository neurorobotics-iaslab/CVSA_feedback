#ifndef FEEDBACK_CVSA_SINGLEARROW_CPP_
#define FEEDBACK_CVSA_SINGLEARROW_CPP_

#include "feedback_cvsa/SingleArrow.h"

namespace feedback {

SingleArrow::SingleArrow(const std::string& wintitle, int nclasses) {
	
	this->engine_ = new neurodraw::Engine(wintitle);
	this->engine_->on_keyboard(&SingleArrow::on_keyboard_event, this);

	this->user_quit_ = false;
	this->nclasses_ = nclasses;

	this->setup();

	// Bind dynamic reconfigure callback
	//this->recfg_callback_type_ = boost::bind(&SingleArrow::on_request_reconfigure, this, _1, _2);
	//this->recfg_srv_.setCallback(this->recfg_callback_type_);
	
}

SingleArrow::~SingleArrow(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;
}


void SingleArrow::setup(void) {

	// set the default thresholds
	this->thresholds_.resize(this->nclasses_);

	// create the graphic elements
	this->cross_   = new neurodraw::Cross(0.3f, 0.05f);
	this->arrow_   = new neurodraw::Arrow(0.2f, 0.1f, true, neurodraw::Palette::darkviolet);
	for(int i = 0; i < this->nclasses_; i++) {
		this->circles_.push_back(new neurodraw::Circle(0.15f, true, neurodraw::Palette::white));
	}

	// set the position of the elements
	this->arrow_->rotate(90.0f);
	this->circles_.at(0)->move(-0.5f, -0.5f);
	this->circles_.at(1)->move(0.5f, -0.5f);
	if (this->nclasses_ > 2) {
		this->circles_.at(2)->move(0.0f, 1.0f);
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

void SingleArrow::reset(void) { 
	for(int i = 0; i < this->nclasses_; i++) {
		this->circles_.at(i)->set_color(neurodraw::Palette::lightgrey);
	}
}

void SingleArrow::show_fixation(void) {
	this->cross_->show();
}

void SingleArrow::show_cue(Direction dir) {

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

void SingleArrow::show_boom(Direction dir) { // must show the circle in the bottom left or right
	
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

void SingleArrow::hide_boom(void) {
	for(int i = 0; i < this->nclasses_; i++) {
		this->circles_.at(i)->set_color(neurodraw::Palette::lightgrey);
	}
}

void SingleArrow::hide_fixation(void) {
	this->cross_->hide();
}

void SingleArrow::hide_cue(void) {
	this->arrow_->hide();
}

void SingleArrow::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

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

bool SingleArrow::set_threshold(float input, Direction dir) {

	switch(dir) {
		case Direction::Leftbottom:
			this->thresholds_[0] = input;
			ROS_INFO("Threshold for Direction::Left changed to: %f", input);
			break;
		case Direction::Rightbottom:
			this->thresholds_[1] = input;
			ROS_INFO("Threshold for Direction::Right changed to: %f", input);
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


/*
void SingleArrow::on_request_reconfigure(rosneuro_config_wheel &config, uint32_t level) {

	if( std::fabs(config.left_threshold - this->thresholds_.at(0)) > 0.00001) {
		this->set_threshold(config.left_threshold, Direction::Left);
	}
	
	if( std::fabs(config.right_threshold - this->thresholds_.at(1)) > 0.00001) {
		this->set_threshold(config.right_threshold, Direction::Right);
	}
}
*/

 } // namespace feedback

#endif