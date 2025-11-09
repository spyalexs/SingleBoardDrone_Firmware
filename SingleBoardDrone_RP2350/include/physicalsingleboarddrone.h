#pragma once

#include <Eigen/Dense>

#include "pico/time.h"

//physical descriptions of the drone...

typedef Eigen::Matrix<float, 8, 6> m8d6;
typedef Eigen::Matrix<float, 8, 7> m8d7;
typedef Eigen::Matrix<float, 3, 1> v3d;
typedef Eigen::Matrix<float, 6, 1> v6d;
typedef Eigen::Matrix<float, 3, 3> m3d;
typedef Eigen::Matrix<float, 13, 13> m13d;
typedef Eigen::Matrix<float, 12, 12> m12d;
typedef Eigen::Matrix<float, 7, 1> v7d;
typedef Eigen::Matrix<float, 8, 1> v8d;
typedef Eigen::Matrix<float, 7, 13> m7d13;
typedef Eigen::Matrix<float, 13, 7> m13d7;
typedef Eigen::Matrix<float, 13, 1> v13d;
typedef Eigen::Matrix<float, 1, 13> m1d13;


typedef Eigen::Quaternionf quat;

class DroneState{
    public:
        absolute_time_t stamp;

        quat rot;
        v3d position;
        v6d twist;

        DroneState(){

        }

        DroneState(v13d vec){
            position = vec.block<3,1>(0,0);
            rot = vec.block<4,1>(3,0);
            twist = vec.block<6,1>(7,0);
        }

        //update the current state from a vector
        void update(v13d vec){
            position = vec.block<3,1>(0,0);
            rot = vec.block<4,1>(3,0);
            twist = vec.block<6,1>(7,0);
        }

        // please note order matters here... override the operator may not be the brighest of ideas but alas...
        // the "previous" state should come before the "current" state
        DroneState operator+(DroneState state_2){
            // this is a body frame rotation... (in my dynamics handling the rotation around the body frame is significantly easier)
            rot = rot * state_2.rot;
            position += state_2.position;
            twist += state_2.twist;

            //careful - never done this before should return its modified self
            return *this;
        }

        float getError(DroneState state_2){
            //calculate the "error" between to drone states
            //for the time being this will be the positional error
            return (position - state_2.position).norm();
        }

        v13d to_vector(){
            return v13d(position[0], position[1], position[2], rot.w(), rot.x(), rot.y(), rot.z(), twist[0], twist[1], twist[2], twist[3], twist[4], twist[5]);
        }
};

class DDroneState{
    public:
        v3d d_rot;
        v3d d_position;
        v6d d_twist;

        DDroneState(DroneState state, v6d D_twist){
            d_position = state.twist.block<3,1>(0,0);
            d_rot = state.twist.block<3,1>(3,0);\
            d_twist = D_twist;
        }

        //allow easy addition and integration of changes in drone state
        DDroneState operator*(float scale){
            d_position = d_position * scale;
            d_twist = d_twist * scale;
            d_rot = d_rot * scale;

            return *this;
        }

        DDroneState operator+ (DDroneState state_2){
            DDroneState d_state;

            d_position = d_position + state_2.d_position;
            d_twist = d_twist + state_2.d_twist;
            d_rot = d_rot + state_2.d_rot;

            return *this;
        }

        DDroneState operator- (DDroneState state_2){
            DDroneState d_state;

            d_position = d_position - state_2.d_position;
            d_twist = d_twist - state_2.d_twist;
            d_rot = d_rot - state_2.d_rot;

            return *this;
        }

        DroneState integrate(float dt){
            DroneState state;

            state.position = d_position * dt;
            state.twist = d_twist * dt;
            state.rot = Eigen::AngleAxisf(d_rot[0] * dt, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(d_rot[1] * dt, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(d_rot[2] * dt, Eigen::Vector3f::UnitZ());

            return state;
        }


        DroneState unit_integrate(){
            DroneState state;

            state.position = d_position;
            state.twist = d_twist;
            state.rot = Eigen::AngleAxisf(d_rot[0], Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(d_rot[1], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(d_rot[2], Eigen::Vector3f::UnitZ());

            return state;
        }

    private:
        DDroneState(){

        }

};

class PhysicalSingeBoardDone{

    public:

        // all units are si standard (m, kg ...)
        PhysicalSingeBoardDone(){
            default_unit << 1.0, 0.0, 0.0;

            com << 0.0, 0.0, 0.0;

            
            mass = .120;

            thruster_positions <<
                0.0923, 0.0382, 0.01, 0.707, 0.0, -0.707, 0.0,
                0.0382, 0.0923, 0.01, 0.707, 0.0, -0.707, 0.0,
                -0.0382, 0.0923, 0.01, 0.707, 0.0, -0.707, 0.0,
                -0.0923, 0.0382, 0.01, 0.707, 0.0, -0.707, 0.0,
                -0.0923, -0.0382, 0.01, 0.707, 0.0, -0.707, 0.0,
                -0.0382, -0.0923, 0.01, 0.707, 0.0, -0.707, 0.0,
                0.0382, -0.0923, 0.01, 0.707, 0.0, -0.707, 0.0,
                0.0923, -0.0382, 0.01, 0.707, 0.0, -0.707, 0.0;

            //pull off diagonal terms from solidworks...? 
            rotational_interia <<
                .25 * mass * drone_radius * drone_radius, 0.0, 0.0,
                0.0, .25 * mass * drone_radius * drone_radius, 0.0,
                0.0, 0.0, 0.5 * mass * drone_radius * drone_radius;


            imu_pos << 0.0, .05, 0.0;
            imu_rot = quat(1.0, 0.0,0.0, 0.0);

            gravitational_acceleration << 0.0, 0.0, -9.815;

            drag_first_order << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1;
            drag_second_order << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            prop_drag_cof << 0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001, -0.001;

            process_noise.setZero();
        }

        //for right now the origin of the drone is the dead center of the top of the pcb.

        //pose of all eight thrusters on the drone in order [x,y,z, w, x, y, z]
        m8d7 thruster_positions;

        //center of mass of the drone ...
        v3d com;

        //center of drag of the drone
        v3d cod;

        //mass of the drone
        float mass;

        //default direction for quaternions...
        v3d default_unit;

        //ehh, I like to think its a disc
        float drone_radius = 0.1;

        //roational interia matrix of the drone
        m3d rotational_interia;

        //imu pose
        v3d imu_pos;
        quat imu_rot;

        //barometer position
        v3d barometer_position;

        //acceleration due to gravity
        v3d gravitational_acceleration;

        //drag coefficents 
        v6d drag_first_order;
        v6d drag_second_order;

        //prop drag cofficent -> why the drone rotates - multiply by angular velocity
        v6d prop_drag_cof;

        //constant representing thrust to angular velocity conversion
        float thrust_constant = 0.001;
        
        //convert the foce to angular velocity
        float convert_force_to_w(float f){
            return sqrt(f) / thrust_constant;
        }

        v8d convert_forces_to_w(v8d f){
            return f.cwiseSqrt() / thrust_constant;
        }
        
        m12d process_noise;

};
