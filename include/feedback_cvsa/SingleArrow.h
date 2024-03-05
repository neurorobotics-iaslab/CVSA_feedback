#ifndef FEEDBACK_CVSA_SINGLEARROW_H_
#define FEEDBACK_CVSA_SINGLEARROW_H_

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

//#include <dynamic_reconfigure/server.h>

//#include "rosneuro_feedback_wheel/WheelConfig.h"

#include <neurodraw/Engine.h>
#include <neurodraw/Arrow.h>
#include <neurodraw/Cross.h>
#include <neurodraw/Circle.h>
#include <neurodraw/EventKey.h>

namespace feedback {

const std::array<float, 4> CueAngle { 
		-45.0, 
		-135.0, 
		90.0,
		0.0
};

//using rosneuro_config_wheel = rosneuro_feedback_wheel::WheelConfig;
//using dyncfg_wheel          = dynamic_reconfigure::Server<rosneuro_config_wheel>;


class SingleArrow {
	
	public:
		enum class Direction {Leftbottom = 0, Rightbottom, Up, Timeout, None};

	public:
		SingleArrow(const std::string& wintitle = "neuroarrow", int nclasses = 2);
		virtual ~SingleArrow(void);

		void setup(void);
		void reset(void);
		bool set_threshold(float input, Direction dir);
		bool set_angle_range(float angle);

		void show_fixation(void);
		void show_cue(Direction dir);
		void show_boom(Direction dir);
		void hide_fixation(void);
		void hide_cue(void);
		void hide_boom(void);
		
		virtual bool configure(void) = 0;
		virtual void run(void) = 0;

	protected:
		virtual void on_keyboard_event(const neurodraw::KeyboardEvent& event);
		float input2angle(float input);
		//void on_request_reconfigure(rosneuro_config_wheel &config, uint32_t level);
		

	protected:
		// Graphic elements
		neurodraw::Engine* 		engine_;
		neurodraw::Cross* 		cross_;
		std::vector<neurodraw::Circle*>	circles_; // depending on the number of classes, lb, rb, up
		neurodraw::Arrow*		arrow_;

		// Default configuration
		int nclasses_;
		bool user_quit_;
		Eigen::VectorXf thresholds_;

		//dyncfg_wheel recfg_srv_;
  		//dyncfg_wheel::CallbackType recfg_callback_type_;
		

};

} // namespace feedback

#endif
