#ifndef FEEDBACK_CVSA_TRAININGARROW_H_
#define FEEDBACK_CVSA_TRAININGARROW_H_

#include <numeric>
#include <array>
#include <random>
#include <ros/ros.h>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <neurochrono/Timer.h>

#include "feedback_cvsa/SingleArrow.h"
#include "feedback_cvsa/TrialSequence.h"
#include "feedback_cvsa/Autopilot.h"


namespace feedback {

struct Events {
	static const int Start 		= 1;
	static const int Fixation 	= 786;
	static const int CFeedback 	= 781;
	static const int Hit 		= 897;
	static const int Miss 		= 898;
	static const int Off 		= 32768;
};

struct Duration {
	int begin;
	int start;
	int fixation;
	int cue;
	int feedback_min;
	int feedback_max;
	int boom;
	int timeout;
	int timeout_on_rest;
	int iti;
	int end;
};

class TrainingArrow : public SingleArrow {

	public:
		enum class Modality {Calibration = 0, Evaluation};

	public:
		TrainingArrow(void);
		virtual ~TrainingArrow(void);

		virtual bool configure(void);
		virtual void run(void);

	protected:
		void setevent(int event);
		void sleep(int msecs);
		Direction class2direction(int eventcue);
		Direction is_target_hit(float input, Direction direction, int elapsed, int duration);
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;

		rosneuro_msgs::NeuroEvent  event_msg_;
		rosneuro_msgs::NeuroOutput inputmsg_;

		TrialSequence trialsequence_;

		std::vector<int> classes_;
		std::vector<int> trials_per_class_;

		Duration duration_;
		Modality modality_;

		// Timer
		neurochrono::timer_msecs timer_;

		float current_input_;
		bool has_new_input_;
		const float rate_ = 100.0f;
		bool show_on_rest_;

};


}


#endif
