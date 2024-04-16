#ifndef FEEDBACK_CONTROLCVSA_H_
#define FEEDBACK_CONTROLCVSA_H_

#include <ros/ros.h>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>

#include "feedback_cvsa/CVSA_layout.h"


namespace feedback {

struct Events {
	static const int Start 		= 1;
	static const int Fixation 	= 786;
	static const int CFeedback 	= 781;
	static const int Hit 		= 897;
	static const int Miss 		= 898;
	static const int Off 		= 32768;
	static const int Command 	= 6000;
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

class ControlCVSA : public CVSA_layout {
	

	public:
		ControlCVSA(void);
		~ControlCVSA(void);

		bool configure(void);
		void run(void);

	protected:
		void setevent(int event);
		void sleep(int msecs);
		int class2direction(int eventcue);
		int is_over_threshold(std::vector<float> input);
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);

	private:
		std::vector<std::vector<float>> str2matrix(const std::string& str);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber sub_;
		ros::Publisher	pub_;

		rosneuro_msgs::NeuroEvent  event_msg_;
		rosneuro_msgs::NeuroOutput inputmsg_;

		std::vector<int> classes_;
		Duration duration_;
		
		std::vector<float> current_input_;
		bool has_new_input_;
		const float rate_ = 256.0f;

};

} // namespace feedback

#endif
