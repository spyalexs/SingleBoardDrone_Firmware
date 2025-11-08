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


void Ekf::propagate_imu_measurement(v7d measurement, v8d control){
    m7d13 measurement_matrix;

    //I believe this one is wrong...
    // measurement_matrix << 
    //     (1 - 2 * previous_state.rot.y() * previous_state.rot.y() - 2 * previous_state.rot.z() * previous_state.rot.z()), 
    //     2 * (previous_state.rot.x() * previous_state.rot.y() - previous_state.rot.w() * previous_state.rot.z()), 
    //     2 * (previous_state.rot.x() * previous_state.rot.z() + previous_state.rot.w() * previous_state.rot.y()), 
    //     2 * (previous_state.rot.y() * previous_state.position[1] + previous_state.rot.z() * previous_state.position[2]),
    //     2 * (-2*previous_state.rot.y() * previous_state.position[0] + previous_state.rot.x() * previous_state.position[1] + previous_state.rot.w() * previous_state.position[2]), 
    //     2 * (-2 * previous_state.rot.z() * previous_state.position[0] - previous_state.rot.w() * previous_state.position[1] + previous_state.rot.x() * previous_state.position[2]),
    //     2 * (-previous_state.rot.z() * previous_state.position[1] + previous_state.rot.y() * previous_state.position[2]),
    //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     2 * (previous_state.rot.x() * previous_state.rot.y() + previous_state.rot.w() * previous_state.rot.z()),
    //     (1 - 2 * previous_state.rot.x() * previous_state.rot.x() - 2 * previous_state.rot.z() * previous_state.rot.z()), 
    //     2 * (previous_state.rot.y() * previous_state.rot.z() - previous_state.rot.w() * previous_state.rot.x()), 
    //     2 * (-2*previous_state.rot.x() * previous_state.position[1] + previous_state.rot.y() * previous_state.position[0] + previous_state.rot.z() * previous_state.position[2]), 
    //     2 * (previous_state.rot.x() * previous_state.position[0] + previous_state.rot.y() * previous_state.position[2]), 
    //     2 * (-2*previous_state.rot.z() * previous_state.position[1] + previous_state.rot.w() * previous_state.position[0] - previous_state.rot.w() * previous_state.position[2]), 
    //     2 * (previous_state.rot.z() * previous_state.position[0] - previous_state.rot.x() * previous_state.position[2]), 
    //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     2 * (previous_state.rot.x() * previous_state.rot.z() - previous_state.rot.w() * previous_state.rot.y()),
    //     2 * (previous_state.rot.y() * previous_state.rot.z() + previous_state.rot.w() * previous_state.rot.x()), 
    //     (1 - 2 * previous_state.rot.x() * previous_state.rot.x() - 2 * previous_state.rot.y() * previous_state.rot.y()), 
    //     2 * (-2*previous_state.rot.x() * previous_state.position[2] + previous_state.rot.z() * previous_state.position[0] + previous_state.rot.w() * previous_state.position[1]), 
    //     2 * (-2*previous_state.rot.y() * previous_state.position[2] - previous_state.rot.w() * previous_state.position[0] + previous_state.rot.z() * previous_state.position[1]), 
    //     2 * (-previous_state.rot.w() * previous_state.position[0] + previous_state.rot.y() * previous_state.position[1]), 
    //     2 * (-previous_state.rot.z() * previous_state.position[0] + previous_state.rot.x() * previous_state.position[1]),  
    //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    v3d com_to_imu = drone->imu_pos - drone->com;

    //get the prediction of the current acceleration
    v3d lin_accel = getDTwist(previous_state, control).block<3,1>(0,0) - drone->gravitational_acceleration;

    //measurement format should be [ax;ay;az;qw;qx;qy;qz]
    measurement_matrix << 
        0.0, 0.0, 0.0,
        2 * (previous_state.rot.y() * lin_accel[1] + previous_state.rot.z() * lin_accel[2]),
        2 * (-2*previous_state.rot.y() * lin_accel[0] + previous_state.rot.x() * lin_accel[1] + previous_state.rot.w() * lin_accel[2]), 
        2 * (-2 * previous_state.rot.z() * lin_accel[0] - previous_state.rot.w() * lin_accel[1] + previous_state.rot.x() * lin_accel[2]),
        2 * (-previous_state.rot.z() * lin_accel[1] + previous_state.rot.y() * lin_accel[2]),
        0.0, 0.0, 0.0, 
        com_to_imu[0] - 2 * previous_state.twist[3] * com_to_imu[0], 
        com_to_imu[1] - 2 * previous_state.twist[4] * com_to_imu[0], 
        com_to_imu[2] - 2 * previous_state.twist[5] * com_to_imu[0], 
        0.0,0.0,0.0,
        2 * (-2*previous_state.rot.x() * lin_accel[1] + previous_state.rot.y() * lin_accel[0] + previous_state.rot.z() * lin_accel[2]), 
        2 * (previous_state.rot.x() * lin_accel[0] + previous_state.rot.y() * lin_accel[2]), 
        2 * (-2*previous_state.rot.z() * lin_accel[1] + previous_state.rot.w() * lin_accel[0] - previous_state.rot.w() * lin_accel[2]), 
        2 * (previous_state.rot.z() * lin_accel[0] - previous_state.rot.x() * lin_accel[2]), 
        0.0, 0.0, 0.0, 
        com_to_imu[0] - 2 * previous_state.twist[3] * com_to_imu[1], 
        com_to_imu[1] - 2 * previous_state.twist[4] * com_to_imu[1], 
        com_to_imu[2] - 2 * previous_state.twist[5] * com_to_imu[1], 
        0.0, 0.0, 0.0,
        2 * (-2*previous_state.rot.x() * lin_accel[2] + previous_state.rot.z() * lin_accel[0] + previous_state.rot.w() * lin_accel[1]), 
        2 * (-2*previous_state.rot.y() * lin_accel[2] - previous_state.rot.w() * lin_accel[0] + previous_state.rot.z() * lin_accel[1]), 
        2 * (-previous_state.rot.w() * lin_accel[0] + previous_state.rot.y() * lin_accel[1]), 
        2 * (-previous_state.rot.z() * lin_accel[0] + previous_state.rot.x() * lin_accel[1]),  
        0.0, 0.0, 0.0,
        com_to_imu[0] - 2 * previous_state.twist[3] * com_to_imu[2], 
        com_to_imu[1] - 2 * previous_state.twist[4] * com_to_imu[2], 
        com_to_imu[2] - 2 * previous_state.twist[5] * com_to_imu[2], 
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    m13d7 kalman_gain = state_error_cov * measurement_matrix.transpose() * (measurement_matrix * state_error_cov * measurement_matrix.transpose() + imu->imu_cov).inverse();

    //predict what the measurement should be based on the dynamics
    v7d predicted_measurement = measurement_matrix * previous_state.to_vector();

    //update the state based on the measurementr and the kalman gain
    previous_state.update(previous_state.to_vector() + kalman_gain * (measurement - predicted_measurement));

    //propagate the state error covariance based on the additional measurement
    state_error_cov = (m13d::Identity() - kalman_gain * measurement_matrix) * state_error_cov;
}

void Ekf::propagate_dynamics(absolute_time_t target_time){
    
    //step the dynamics until the reached the target time
    while(previous_state.stamp + current_dynamic_timestep * 10e6 < target_time){
        step_dynamics();
    }

    //final step to reach exactly target time
    current_dynamic_timestep = float(target_time - previous_state.stamp) / 10e6;
    step_dynamics();

    //now we are caught up to the target time...
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
         .025);

    //propagate the covariance
    m12d state_transition_mat = getJacobian(previous_state) * current_dynamic_timestep + Eigen::Matrix<float,12,12>::Identity();
    state_error_cov = state_transition_mat * state_error_cov * state_transition_mat.transpose() + drone->process_noise;
}

v6d Ekf::getDTwist(DroneState current_state, v8d control){
    //get the change in velocity based on the current state and the control inputs

    //get the body frame forces due to the control inputs
    v6d thrust_effect = thruster_effect * control;

    //get the body frame forces and torques due to drag
    v6d drag_effect = -drone->drag_first_order *  current_state.twist + -drone->drag_second_order * current_state.twist * current_state.twist.cwiseAbs();

    //since the angular velocities will be minial, it is assumed that any effects caused by the angular drags being offset from the com is neglidable
    v3d offset_drag_effect = (drone->cod - drone->com).cross(drag_effect.block<3,1>(0,0));
    drag_effect(4) += offset_drag_effect(0);
    drag_effect(5) += offset_drag_effect(1);
    drag_effect(6) += offset_drag_effect(2);

    //get the linear accelerations in world frame...
    v3d lin_accel = (current_state.rot * (thrust_effect.block<3,1>(0,0) + drag_effect.block<3,1>(0,0)) / drone->mass + drone->gravitational_acceleration);

    //get the effect caused by prop drag
    //NOTE this torque is not effected by the COM being offset from center of the drone... it balanaces out... (Pretty easy to conceptualize)
    v3d prop_drag_effect(0.0, 0.0, drone->convert_forces_to_w(control).dot(drone->prop_drag_cof));

    //centripical effects
    //for the sake of the centripital effects, making the assumption that the drone is a homogenous disk, therefore the com is the center of the disk
    //this gets nasty if the above assumption does not hold
    v3d centripical_effects(
        (drone->rotational_interia(2,2) - drone->rotational_interia(3,3)) / drone->rotational_interia(1,1) * current_state.twist(4) * current_state.twist(5),
        (drone->rotational_interia(3,3) - drone->rotational_interia(1,1)) / drone->rotational_interia(2,2) * current_state.twist(3) * current_state.twist(4),
        (drone->rotational_interia(1,1) - drone->rotational_interia(2,2)) / drone->rotational_interia(3,3) * current_state.twist(3) * current_state.twist(5)
    );

    //"body frame rotaitons"
    v3d rot_accel = (thrust_effect.block<3,1>(3,0) + drag_effect.block<3,1>(3,0) + prop_drag_effect).cwiseQuotient(v3d(drone->rotational_interia(1,1), drone->rotational_interia(2,2), drone->rotational_interia(3,3))) - centripical_effects;

    //im attempting to vertically concatenate here...
    return lin_accel, rot_accel;
}

m13d Ekf::getJacobian(DroneState current_state){
    //should hopefully only be used to propagate covariance...

    //get the change in velocity based on the current state
    m13d jacobian;
    jacobian.setZero();

    //velocities are in world frame and so are the positions so this is simple
    jacobian(0, 7) = 1;
    jacobian(1, 8) = 1;
    jacobian(2, 9) = 1;

    //rotations are in the body frame and so are the rotation velocities (this is more abstract)
    jacobian(3, 10) = -previous_state.rot.x();
    jacobian(3, 11) = -previous_state.rot.y();
    jacobian(3, 12) = -previous_state.rot.z();
    jacobian(4, 10) = previous_state.rot.w();
    jacobian(4, 11) = -previous_state.rot.z();
    jacobian(4, 12) = previous_state.rot.y();
    jacobian(5, 10) = previous_state.rot.z();
    jacobian(5, 11) = previous_state.rot.w();
    jacobian(5, 12) = -previous_state.rot.x();
    jacobian(6, 10) = -previous_state.rot.y();
    jacobian(6, 11) = previous_state.rot.z();
    jacobian(6, 12) = previous_state.rot.w();

    //quaternion derivative with respect to the current quaternion
    jacobian(3,4) = -current_state.twist[3];
    jacobian(3,5) = -current_state.twist[4];
    jacobian(3,6) = -current_state.twist[5];
    jacobian(4,3) = current_state.twist[3];
    jacobian(4,5) = current_state.twist[5];
    jacobian(4,6) = -current_state.twist[4];
    jacobian(5,3) = current_state.twist[4];
    jacobian(5,4) = -current_state.twist[5];
    jacobian(5,6) = current_state.twist[3];
    jacobian(6,3) = current_state.twist[5];
    jacobian(6,4) = current_state.twist[4];
    jacobian(6,5) = -current_state.twist[3];

    //drag with respect to rate of velocity change
    v6d drag_derivative = drone->drag_first_order + -drone->drag_second_order * current_state.twist.cwiseAbs();
    v3d world_drag_derivative = current_state.rot * drag_derivative.block<3,1>(0,0);
    jacobian(7, 7) = world_drag_derivative(0);
    jacobian(8, 8) = world_drag_derivative(1);
    jacobian(9, 9) = world_drag_derivative(2);

    //do need the euler angles here
    v3d ea = current_state.rot.toRotationMatrix().eulerAngles(2,1,0);
    float y = ea[0];
    float p = ea[1];
    float r = ea[2];

    // linear drags with respect to rate of angular change (also gravity...)
    v3d current_linear_drag = drag_derivative.block<3,1>(0,0).cwiseProduct(current_state.twist.block<3,1>(0,0));
    jacobian(7, 4) = 2 * (previous_state.rot.y() * current_linear_drag[1] + previous_state.rot.z() * current_linear_drag[2]);
    jacobian(7, 5) = 2 * (-2*previous_state.rot.y() * current_linear_drag[0] + previous_state.rot.x() * current_linear_drag[1] + previous_state.rot.w() * current_linear_drag[2]);
    jacobian(7, 6) = 2 * (-2 * previous_state.rot.z() * current_linear_drag[0] - previous_state.rot.w() * current_linear_drag[1] + previous_state.rot.x() * current_linear_drag[2]);
    jacobian(7, 3) = 2 * (-previous_state.rot.z() * current_linear_drag[1] + previous_state.rot.y() * current_linear_drag[2]);
    jacobian(8,4) = 2 * (-2*previous_state.rot.x() * current_linear_drag[1] + previous_state.rot.y() * current_linear_drag[0] + previous_state.rot.z() * current_linear_drag[2]);
    jacobian(8,5) = 2 * (previous_state.rot.x() * current_linear_drag[0] + previous_state.rot.y() * current_linear_drag[2]);
    jacobian(8,6) = 2 * (-2*previous_state.rot.z() * current_linear_drag[1] + previous_state.rot.w() * current_linear_drag[0] - previous_state.rot.w() * current_linear_drag[2]);
    jacobian(8,3) = 2 * (previous_state.rot.z() * current_linear_drag[0] - previous_state.rot.x() * current_linear_drag[2]);
    jacobian(9,4) = 2 * (-2*previous_state.rot.x() * current_linear_drag[2] + previous_state.rot.z() * current_linear_drag[0] + previous_state.rot.w() * current_linear_drag[1]);
    jacobian(9,5) = 2 * (-2*previous_state.rot.y() * current_linear_drag[2] - previous_state.rot.w() * current_linear_drag[0] + previous_state.rot.z() * current_linear_drag[1]);
    jacobian(9,6) = 2 * (-previous_state.rot.w() * current_linear_drag[0] + previous_state.rot.y() * current_linear_drag[1]);
    jacobian(9,3) = 2 * (-previous_state.rot.z() * current_linear_drag[0] + previous_state.rot.x() * current_linear_drag[1]);

    //angular drag derivatives
    jacobian(10, 10) = drag_derivative(3);
    jacobian(11, 11) = drag_derivative(4);
    jacobian(12, 12) = drag_derivative(5);

    //centripical effects partials
    jacobian(10,11) = (drone->rotational_interia(2,2) - drone->rotational_interia(3,3)) / drone->rotational_interia(1,1) * current_state.twist(5);
    jacobian(10,12) = (drone->rotational_interia(2,2) - drone->rotational_interia(3,3)) / drone->rotational_interia(1,1) * current_state.twist(4);
    jacobian(11,10) = (drone->rotational_interia(3,3) - drone->rotational_interia(1,1)) / drone->rotational_interia(2,2) * current_state.twist(5);
    jacobian(11,12) = (drone->rotational_interia(3,3) - drone->rotational_interia(1,1)) / drone->rotational_interia(2,2) * current_state.twist(3);
    jacobian(12,10) = (drone->rotational_interia(1,1) - drone->rotational_interia(2,2)) / drone->rotational_interia(3,3) * current_state.twist(4);
    jacobian(12,11) = (drone->rotational_interia(1,1) - drone->rotational_interia(2,2)) / drone->rotational_interia(3,3) * current_state.twist(3);
}

void Ekf::propagate_gps_measurement(){

}

void Ekf::propagate_barometer_measurement(){

}