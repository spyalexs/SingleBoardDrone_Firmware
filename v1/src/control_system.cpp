#include "control_system.h"

ControlSystem::ControlSystem(){

    //set curve parameters
    force_to_current_curve = FORCE_TO_CURRENT_COFS;
    current_to_pwm_curve = CURRENT_TO_PWM_COFS;
}

void ControlSystem::setDesiredForces(float thrust, float roll_t, float pitch_t, float yaw_t){

    //must convert the desired thrust, roll torque, pitch torque and yaw torque.
    //all in body frame

    

}

void ControlSystem::tic(){
    
    //calculate the current and the pwm required
    for(int i = 0; i < 8; i++){

        //currents
        desired_currents[i] = convertForceToCurrent(desired_forces[i]);

        //open loop pwms
        ff_pwm[i] = convertCurrentToPWM(desired_currents[i]);
    }
}


float ControlSystem::convertForceToCurrent(float force){

    //add in a correct curve fit here with tune
    return force * force_to_current_curve.at(0);

}

int ControlSystem::convertCurrentToPWM(float current){

    //add in a correct curve fit here with tune
    return int(current * current_to_pwm_curve.at(0));

}