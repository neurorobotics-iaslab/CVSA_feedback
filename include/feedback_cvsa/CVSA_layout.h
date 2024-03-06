#ifndef FEEDBACK_CVSA_LAYOUT_H_
#define FEEDBACK_CVSA_LAYOUT_H_

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <dynamic_reconfigure/server.h>

#include "feedback_cvsa/CVSAConfig.h"

#include <neurodraw/Engine.h>
#include <neurodraw/Arrow.h>
#include <neurodraw/Cross.h>
#include <neurodraw/Circle.h>
#include <neurodraw/EventKey.h>

namespace feedback {

const std::array<float, 4> CueAngle { 
        45.0, 
        135.0, 
        -90.0,
        0.0
};


using config_cvsa          = feedback_cvsa::CVSAConfig;
using dyncfg_cvsa          = dynamic_reconfigure::Server<config_cvsa>;


class CVSA_layout {
    
    public:
        enum class Direction {Leftbottom = 0, Rightbottom, Up, Timeout, None};

    public:
        CVSA_layout(const std::string& wintitle = "cvsa");
        virtual ~CVSA_layout(void);

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
        void on_request_reconfigure(config_cvsa &config, uint32_t level);
        

    protected:
        // Graphic elements
        neurodraw::Engine*         engine_;
        neurodraw::Cross*         cross_;
        std::vector<neurodraw::Circle*>    circles_; // depending on the number of classes, lb, rb, up
        neurodraw::Arrow*        arrow_;

        // Default configuration
        int nclasses_;
        bool user_quit_;
        Eigen::VectorXf thresholds_;

        dyncfg_cvsa recfg_srv_;
        dyncfg_cvsa::CallbackType recfg_callback_type_;
        

};

} // namespace feedback

#endif
