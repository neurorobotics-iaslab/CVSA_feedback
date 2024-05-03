#ifndef FEEDBACK_CVSA_LAYOUT_H_
#define FEEDBACK_CVSA_LAYOUT_H_

#include <ros/ros.h>

#include "feedback_cvsa/CVSAConfig.h"

#include <neurodraw/Engine.h>
#include <neurodraw/Palette.h>
#include <neurodraw/Rectangle.h>
#include <neurodraw/Cross.h>
#include <neurodraw/Circle.h>
#include <neurodraw/Ring.h>
#include <neurodraw/EventKey.h>

namespace feedback {

const std::array<neurodraw::Color, 5> CuePalette { 
		neurodraw::Palette::royalblue, 
		neurodraw::Palette::firebrick, 
		neurodraw::Palette::orange,
		neurodraw::Palette::darkgray,
        neurodraw::Palette::yellow // for timeout
};

class CVSA_layout {

    public:
        CVSA_layout(const std::string& wintitle = "cvsa");
        virtual ~CVSA_layout(void);

        void setup(void);
        void reset(void);
        bool set_nclasses(int nclasses);
        bool set_circle_positions(std::vector<std::vector<float>> circlePositions);

        void show_fixation(void);
        void show_calibration(std::vector<float> position);
        void show_center(void);
        void show_rings_classes(void);
        void show_cue(int index);
        void show_boom(int idx_position, int idx_color);
        void hide_fixation(void);
        void hide_calibration(void);
        void hide_center(void);
        void hide_cue(void);
        void hide_boom(void);
        
        virtual bool configure(void) = 0;
        virtual void run(void) = 0;

    protected:
        virtual void on_keyboard_event(const neurodraw::KeyboardEvent& event);
        float input2angle(float input);
        

    protected:
        // Graphic elements
        neurodraw::Engine*                 engine_;
        neurodraw::Cross*                  cross_;
        std::vector<neurodraw::Ring*>      rings_; // depending on the number of classes, lb, rb, up
        neurodraw::Rectangle*              square_;
        neurodraw::Circle*                 circle_; // to show the hit
        neurodraw::Circle*                 center_; // to show the center point to look
        neurodraw::Circle*                 calibration_; // to show the calibration point to look

        // Default configuration
        int              nclasses_;
        bool             user_quit_;
        std::vector<std::vector<float>> circlePositions_;
        

};

} // namespace feedback

#endif
