#include "ekf.h"

Ekf::Ekf(std::shared_ptr<PhysicalSingeBoardDone> Drone){

    drone = Drone;

    load_in_parameters();
}


void Ekf::load_in_parameters(){

    //calculate force effect matrix
    thruster_effect.setZero();
    for(int i = 0; i < 8; i++){
        //go by the thruster
        Eigen::Quaternionf quat(drone->thruster_positions(i, 3), drone->thruster_positions(i, 4), drone->thruster_positions(i, 5), drone->thruster_positions(i, 6));
        //the unit thrust vector produced by the thruster
        v3d thrust_effect = quat * drone->default_unit;

        //the torque produced by a unit thrust vector about the com
        v3d torque_effect(drone->thruster_positions(i, 0), drone->thruster_positions(i, 1), drone->thruster_positions(i, 2));
        torque_effect = torque_effect - drone->com; // must take into account the com
        torque_effect = torque_effect.cross(thrust_effect);

        thruster_effect(i,0) = thrust_effect[0];
        thruster_effect(i,1) = thrust_effect[1];
        thruster_effect(i,2) = thrust_effect[2];

        thruster_effect(i,3) = torque_effect[0];
        thruster_effect(i,4) = torque_effect[1];
        thruster_effect(i,5) = torque_effect[2];
    }
}


void Ekf::propagate_imu_measurement(){
    
}

void Ekf::propagate_dynamics(){

}

void Ekf::step_dynamics(){
    //evaluate the dynamics at various points

    //RKF45
    DDroneState k1 = DDroneState(previous_state, getDTwist(previous_state, previous_control)) * current_dynamic_timestep;
    DroneState d_k_1 = k1.integrate(.25);
    DDroneState k2 = DDroneState(previous_state + d_k_1, getDTwist(previous_state + d_k_1, previous_control)) * current_dynamic_timestep;
    DroneState d_k_2 = (k1 * .09375 + k2 * .28125).unit_integrate();
    DDroneState k3 = DDroneState(previous_state + d_k_2, getDTwist(previous_state + d_k_2, previous_control)) * current_dynamic_timestep;
    DroneState d_k_3 = (k1 * .879380974 - k2 * 3.277196177 + k3 * 3.320892126).unit_integrate();
    DDroneState k4 = DDroneState(previous_state + d_k_3, getDTwist(previous_state + d_k_3, previous_control)) * current_dynamic_timestep;
    DroneState d_k_4 = (k1 * 2.032407407 - k2 * 8 + k3 * 7.173489279 - k4 * 0.205896686).unit_integrate();
    DDroneState k5 = DDroneState(previous_state + d_k_4, getDTwist(previous_state + d_k_4, previous_control)) * current_dynamic_timestep;
    DroneState d_k_5 = (k1 * -0.296296296 + k2 * 2 - k3 * 1.381676413 + k4 * 0.45297271 - k5 * 0.275).unit_integrate();
    DDroneState k6 = DDroneState(previous_state + d_k_5, getDTwist(previous_state + d_k_5, previous_control)) * current_dynamic_timestep;

    //fourth order iterate state
    previous_state = previous_state + (k1 * 0.115740741 + k3 * 0.548927875 + k4 * 0.535722994 - k5 * 0.2).unit_integrate();
    previous_state.stamp += current_dynamic_timestep;

    //get fifth order est for error correction
    DroneState est_5 = previous_state + (k1 * 0.118518519 + k3 * 0.518986355 + k4 * 0.50613149 - k5 * 0.18 + k6 * 0.03636363).unit_integrate();

    //calculate the optimal step size
    current_dynamic_timestep = current_dynamic_timestep * powf(
        (dynamic_error_tolerance * current_dynamic_timestep) / (2*est_5.getError(previous_state)),
         .025)
}

v6d Ekf::getDTwist(DroneState current_state, v8d control){
    //get the change in velocity based on the current state and the control inputs

    //get the body frame forces due to the control inputs
    v6d thrust_effect = thruster_effect * control;

    //get the body frame forces and torques due to drag
    v6d drag_effect = -drone->drag_first_order *  current_state.twist + -drone->drag_second_order * current_state.twist * current_state.twist.cwiseAbs();

    //get the linear accelerations in world frame...
    v3d lin_accel = (current_state.rot * (thrust_effect.block<3,1>(0,0) + drag_effect.block<3,1>(0,0)) + drone->gravitational_acceleration) / drone->mass;

    //get the effect caused by prop drag
    v3d prop_drag_effect(0.0, 0.0, drone->convert_forces_to_w(control).dot(drone->prop_drag_cof));

    //centripical effects
    v3d centripical_effects(
        (drone->rotational_interia(2,2) - drone->rotational_interia(3,3)) / drone->rotational_interia(1,1) * current_state.twist(2) * current_state.twist(3),
        (drone->rotational_interia(3,3) - drone->rotational_interia(1,1)) / drone->rotational_interia(2,2) * current_state.twist(1) * current_state.twist(3),
        (drone->rotational_interia(1,1) - drone->rotational_interia(2,2)) / drone->rotational_interia(3,3) * current_state.twist(1) * current_state.twist(2)
    );

    //"body frame rotaitons"
    v3d rot_accel = (thrust_effect.block<3,1>(3,0) + drag_effect.block<3,1>(3,0) + prop_drag_effect).cwiseQuotient(v3d(drone->rotational_interia(1,1), drone->rotational_interia(2,2), drone->rotational_interia(3,3))) - centripical_effects;

    //im attempting to vertically concatenate here...
    return lin_accel, rot_accel;
}

void Ekf::propagate_gps_measurement(){

}

void Ekf::propagate_barometer_measurement(){

}