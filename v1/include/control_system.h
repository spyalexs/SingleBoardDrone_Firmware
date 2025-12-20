#pragma once

#include <Eigen/Dense>

#include "pico/time.h"

#include "physicalsingleboarddrone.h"

#define FORCE_TO_CURRENT_COFS {0.1}
#define CURRENT_TO_PWM_COFS {1.0}


class ControlSystem{

    public:
        ControlSystem();

        void setDesiredForces();

        void tic();

    private: 

        float convertForceToCurrent(float force);
        int convertCurrentToPWM(float current);

        //newtons
        v8d desired_forces;
        //amps
        v8d desired_currents;
        //pwm runs from 0 to 1
        v8i ff_pwm;

        std::vector<float> force_to_current_curve;
        std::vector<float> current_to_pwm_curve;


};