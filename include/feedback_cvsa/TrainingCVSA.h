#ifndef FEEDBACK_CVSA_TRAININGCVSA_H_
#define FEEDBACK_CVSA_TRAININGCVSA_H_

#include <numeric>
#include <array>
#include <ros/ros.h>
#include <random>

#include <dynamic_reconfigure/server.h>
#include "feedback_cvsa/Repeat_trial.h"

#include "feedback_cvsa/Trials_to_keep.h"
#include <std_srvs/Trigger.h>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <neurochrono/Timer.h>

#include "feedback_cvsa/CVSA_layout.h"
#include "feedback_cvsa/TrialSequence.h"


#include "feedback_cvsa/Autopilot.h"

#include <numeric>
#include <algorithm>

#include <sndfile.h>
#include <ao/ao.h>


namespace feedback {

struct Events {
    static const int Start         = 1;
    static const int Fixation      = 786;
    static const int CFeedback     = 781;
    static const int Hit           = 897;
    static const int Miss          = 898;
    static const int Timeout       = 899;
    static const int Off           = 32768;

    static const int Fake_rest          = 784;

    static const int StartCalibEye = 2;
    
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
    int calibration;
};

using config_cvsa          = feedback_cvsa::CVSAConfig;
using dyncfg_cvsa          = dynamic_reconfigure::Server<config_cvsa>;

class TrainingCVSA : public CVSA_layout {

    public:
        enum class Modality {Calibration = 0, Evaluation};

    public:
        TrainingCVSA(void);
        virtual ~TrainingCVSA(void);

        virtual bool configure(void);
        virtual void run(void);

    protected:
        void eye_calibration(void);
        void bci_protocol(void);
        void setevent(int event);
        void sleep(int msecs);
        int class2direction(int eventcue);
		float direction2threshold(int index);
        int class2index(int eventcue);
        int is_target_hit(std::vector<float> input, int elapsed, int duration);
        void on_received_data(const rosneuro_msgs::NeuroOutput& msg);
        void on_request_reconfigure(config_cvsa &config, uint32_t level);
        bool on_repeat_trial(feedback_cvsa::Repeat_trial::Request &req, feedback_cvsa::Repeat_trial::Response &res);
        void loadWAVFile(const std::string& filename);
        void openAudioDevice(void);
        void closeAudioDevice(void);
        void fillAudioBuffer(int& idx_sampleAudio, const size_t& sampleAudio, bool cue);
        void setAudio(int& idx_sampleAudio, size_t& sampleAudio, size_t& bufferAudioSize, size_t& n_sampleAudio);

    private:
        std::vector<std::vector<float>> str2matrix(const std::string& str);
        std::vector<float> normalize4audio(std::vector<float>& input);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::Publisher pub_trials_keep_;
        ros::ServiceServer srv_repeat_trial_;
        ros::ServiceClient srv_face_detection_ready_;
        ros::ServiceClient srv_robot_moving_;
        ros::ServiceClient srv_imu_;

        rosneuro_msgs::NeuroEvent  event_msg_;
        rosneuro_msgs::NeuroOutput inputmsg_;

        feedback::TrialSequence trialsequence_;

        std::vector<int> classes_;
        std::vector<int> trials_per_class_;
        std::vector<int> max_trials_per_class_;

        Duration duration_;
        Modality modality_;
        int mindur_active_;
        int maxdur_active_;

        // Timer
        neurochrono::timer_msecs timer_;

        std::vector<float> current_input_;
        const float rate_ = 100.0f;
        bool show_on_rest_;
        std::vector<float> thresholds_;
		bool eye_calibration_;
        bool eye_detection_;
        std::vector<std::vector<float>> calibration_positions_;
        std::vector<int> calibration_classes_;
        int trial_ok_;
        std::vector<int> trials_keep_;
        bool eye_motion_online_;

        dyncfg_cvsa recfg_srv_;
        dyncfg_cvsa::CallbackType recfg_callback_type_;

        // feedback audio
        std::string audio_path_;
        int channels_audio_;
        int sampleRate_audio_;
        std::vector<short> buffer_audio_full_;
        std::vector<short> buffer_audio_played_;
        ao_device *device_audio_;
        std::vector<float> init_percentual_;

        // for positive feedback
        bool positive_feedback_ = false;

        // for robot control
        bool robot_control_ = false;

        // for audio cue
        bool audio_cue_ = false;

        // for fake rest
        bool fake_rest_ = false;

        // for imu
        bool imu_ = false;
};


}


#endif
