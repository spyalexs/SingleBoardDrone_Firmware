#pragma once

#include <Eigen/Dense>
#include <memory>
#include <cmath>

#include "physicalsingleboarddrone.h"
#include "imu.h"



class Ekf{
    public:
        Ekf(std::shared_ptr<PhysicalSingeBoardDone> Drone);

        DroneState getCurrentState();

    private:
        void load_in_parameters();

        m8d6 thruster_effect;

        std::shared_ptr<PhysicalSingeBoardDone> drone;
        std::shared_ptr<IMU> imu;

        DroneState previous_state;
        v8d previous_control;
        m13d state_error_cov;

        float current_dynamic_timestep = 0.01;
        float dynamic_error_tolerance = .000001;

        void step_dynamics();

        void propagate_imu_measurement();
        void propagate_dynamics(absolute_time_t target_time);
        void propagate_gps_measurement();
        void propagate_barometer_measurement();

        v6d getDTwist(DroneState current_state, v8d control);
        m13d getJacobian(DroneState current_state);

        
        

    

};