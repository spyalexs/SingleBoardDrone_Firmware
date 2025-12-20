#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "pico/time.h"

#include "sbd_descriptions.hpp"

typedef Eigen::Matrix<float, 2, 1> V2f;
typedef Eigen::Matrix<float, 3, 1> V3f;
typedef Eigen::Matrix<float, 4, 1> V4f;
typedef Eigen::Matrix<float, 5, 1> V5f;
typedef Eigen::Matrix<float, 6, 1> V6f;
typedef Eigen::Matrix<float, 7, 1> V7f;
typedef Eigen::Matrix<float, 8, 1> V8f;
typedef Eigen::Matrix<float, 3, 3> M3f3;
typedef Eigen::Matrix<float, 8, 6> M8f6;
typedef Eigen::Matrix<float, 6, 8> M6f8;

typedef Eigen::Quaternion<float> Quat;

class DDroneState;

class DroneModel{
    public:
        DroneModel();



        M6f8 thruster_effect;

        //inherited parameters from descriptions file
        float mass;
        float thrust_constant;
        V3f com;
        V3f cod;
        V6f first_order_drag_cofs;
        V6f second_order_drag_cofs;
        V8f prop_drag;
        V3f gravitational_acceleration;
        M3f3 rotational_interia;

        V8f convert_forces_to_w(V8f f);

};

class DroneState{
    public:
        DroneState();
    
        void update_rk4(float dt, V8f* control);

        void update_rk45(float dt, V8f* control);


        //for rk45 integration
        float current_dynamic_timestep = 0;

        DroneModel* model;

        V6f twist;
        V3f position;
        Quat rotation;

        void update(DDroneState ds, float dt);
        void update(DDroneState* ds, float dt);
        void update(std::vector<DDroneState*> dsv, std::vector<float> weights, float dt);

        absolute_time_t stamp;

        float get_error(DroneState* state2);
        
        DroneState get_increment(DDroneState *ds, float dt);
        DroneState get_increment(std::vector<DDroneState*> dsv, std::vector<float> weights, float dt);
};

class DDroneState{
    public:
        DDroneState(DroneState* state, V8f* control);
        DDroneState(DroneState state, V8f* control);
        DDroneState(V3f dlv_1, V3f dlv_2, V3f dav_1, V3f dav_2, V3f dp_1, V3f dp_2, V3f dr_1, V3f dr_2);

        //@brief around the body frame axis
        V3f d_lin_vel;

        //@brief around the body frame axis
        V3f d_ang_vel;

        //@brief world frame position
        V3f d_pos;

        //@brief relative to the body frame
        V3f d_rot;

        //define operators
        DDroneState operator+ (DDroneState state_2){
            return DDroneState(d_lin_vel, state_2.d_lin_vel, d_ang_vel, state_2.d_ang_vel, d_pos, state_2.d_pos, d_rot, state_2.d_rot);
        }

        DDroneState operator* (float k){

            d_lin_vel *= k;
            d_ang_vel *= k;
            d_pos *= k;
            d_rot *= k;

            return *this;
        }  
        
        DDroneState operator/ (float k){

            d_lin_vel /= k;
            d_ang_vel /= k;
            d_pos /= k;
            d_rot /= k;

            return *this;
        }  

};