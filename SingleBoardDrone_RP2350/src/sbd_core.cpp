#include "sbd_core.hpp"


DroneModel::DroneModel(){

    //inherit some paramters
    mass = SBD_MASS;
    com = SBD_COM;
    cod = SBD_COD;
    first_order_drag_cofs = SBD_FIRST_DRAG;
    second_order_drag_cofs = SBD_SECOND_DRAG;
    prop_drag = SDB_PROP_DRAG;
    gravitational_acceleration = GRAVITY_VECTOR;
    rotational_interia << SBD_ROT_INTERTIA;
    thrust_constant = SBD_THRUST_CONSTANT;

    //initialize the thruster control effects
    std::vector<V7f> thruster_poses = THRUST_POSES;

    //calculate force effect matrix
    thruster_effect.setZero();
    for(int i = 0; i < thruster_poses.size(); i++){
        //go by the thruster
        Eigen::Quaternionf quat(thruster_poses.at(i)[3], thruster_poses.at(i)[4], thruster_poses.at(i)[5], thruster_poses.at(i)[6]);
        //the unit thrust vector produced by the thruster
        V3f thrust_effect = quat * UNIT_THRUST;

        //the torque produced by a unit thrust vector about the com
        V3f torque_effect(thruster_poses.at(i)[0], thruster_poses.at(i)[1], thruster_poses.at(i)[2]);
        torque_effect = torque_effect - com; // must take into account the com
        torque_effect = torque_effect.cross(thrust_effect);

        thruster_effect(0,i) = thrust_effect[0];
        thruster_effect(1,i) = thrust_effect[1];
        thruster_effect(2,i) = thrust_effect[2];

        thruster_effect(3,i) = torque_effect[0];
        thruster_effect(4,i) = torque_effect[1];
        thruster_effect(5,i) = torque_effect[2];
    }
}

V8f DroneModel::convert_forces_to_w(V8f f){
    return f.cwiseSqrt() / thrust_constant;
}


DroneState::DroneState(){
    //set the deafult dynamic step size
    current_dynamic_timestep = DEFAULT_DYANMICS_STEP_SIZE / float(1000000);

    twist = V6f::Zero();
    position = V3f::Zero();
    rotation = Quat(1.0, 0.0, 0.0, 0.0);
}

void DroneState::update(DDroneState ds, float dt){

    //update the velocity
    twist.head<3>() += ds.d_lin_vel * dt;
    twist.tail<3>() += ds.d_ang_vel * dt;

    //update the position
    position += ds.d_pos * dt;

    //convert to delta quaternion
    float theta = (ds.d_rot * dt).norm();
    float k = sin(theta / 2) * dt;
    Quat dq(cos(theta / 2), k * ds.d_rot[0], k * ds.d_rot[1], k * ds.d_rot[2]);

    //update the rotation
    rotation = rotation * dq;
}

void DroneState::update(DDroneState* ds, float dt){

    //update the velocity
    twist.head<3>() += ds->d_lin_vel * dt;
    twist.tail<3>() += ds->d_ang_vel * dt;

    //update the position
    position += ds->d_pos * dt;

    //convert to delta quaternion
    float theta = (ds->d_rot * dt).norm();
    float k = sin(theta / 2) * dt;
    Quat dq(cos(theta / 2), k * ds->d_rot[0], k * ds->d_rot[1], k * ds->d_rot[2]);

    //update the rotation
    rotation = rotation * dq;
}

void DroneState::update(std::vector<DDroneState*> dsv, std::vector<float> weights, float dt){
    if(dsv.size() == 0){
        return;
    }

    DDroneState weighted_update = (*dsv.at(0)) * weights.at(0);

    for(int i = 1;  i < dsv.size(); i++){
        weighted_update = weighted_update + *dsv.at(i) * weights.at(i);
    }

    update(&weighted_update, dt);
}


DroneState DroneState::get_increment(DDroneState* ds, float dt){
    
    DroneState incremented_state = *this;
    incremented_state.update(ds, dt);

    return incremented_state;
}

DroneState DroneState::get_increment(std::vector<DDroneState*> dsv, std::vector<float> weights, float dt){
    
    if(dsv.size() == 0){
        return *this;
    }

    DDroneState weighted_update = (*dsv.at(0)) * weights.at(0);

    for(int i = 1;  i < dsv.size(); i++){
        //printf("Weighted Update %f %f %f weight: %f d_vz: %f\n", weighted_update.d_lin_vel[0],  weighted_update.d_lin_vel[1],  weighted_update.d_lin_vel[2], weights.at(i-1), dsv.at(i-1)->d_lin_vel[2]);
        weighted_update = weighted_update + (*dsv.at(i)) * weights.at(i);
    }

    //printf("Weighted Update %f %f %f  weight: %f  d_vz: %f\n", weighted_update.d_lin_vel[0],  weighted_update.d_lin_vel[1],  weighted_update.d_lin_vel[2], weights.back(), dsv.back()->d_lin_vel[2]);

    DroneState incremented_state = *this;
    incremented_state.update(&weighted_update, dt);

    return incremented_state;
}

/// @brief Intelligently step the dyanmics
/// @param step_size the step to propagate the dynamics (microseconds)
void DroneState::step_dynamics(uint64_t step_size){
    
    //set the target stamp
    uint64_t target_time = stamp + step_size;

    //update in rk45 steps until the final step
    while(stamp + current_dynamic_timestep < target_time){
        update_rk45(&current_control);
    }

    //resize the final step -- will always be a small step than recommended by rk45
    current_dynamic_timestep = target_time - stamp;
    update_rk45(&current_control);
}

/// @brief step the dyamics using a ficed timetep
/// @param step_size total time to populate the dynamics(microseconds) 
/// @param increment each individual update time (microseconds)
void DroneState::step_dynamics_rigid(uint64_t step_size, uint64_t increment = DEFAULT_DYANMICS_STEP_SIZE){
    uint64_t target_time = stamp + step_size;

    while(stamp + increment < target_time){
        update_rk4(increment, &current_control);
    }

    update_rk4(target_time - stamp, &current_control);
}


void DroneState::update_rk4(float dt, V8f* control){

    //update using runge-kutta 4
    DDroneState k1(this, control);
    DDroneState k2(this->get_increment(&k1, dt / 2), control);
    DDroneState k3(this->get_increment(&k2, dt / 2), control);
    DDroneState k4(this->get_increment(&k3, dt), control);

    this->update((k1 + k2*2 + k3*2 + k4) / 6, dt);

    //update the stamp
    stamp += dt * 1e6;
}

float DroneState::get_error(DroneState* state2){
    float error = (position - state2->position).norm();

    if(fabs(error) < 1e-10){
        return 1e-10;
    }

    return error;
}

void DroneState::update_rk45(V8f* control){

    //the timestep before the update
    float previous_timestep = current_dynamic_timestep;

    //copy the current twist incase of a revert
    V6f previous_twist = twist;
    V3f previous_position = position;
    Quat previous_rotation = rotation;

    DDroneState k1(this, control);
    DDroneState k2(this->get_increment(&k1, current_dynamic_timestep / 4), control);
    //printf("k1: %f\n", k1.d_lin_vel[2]);
    DDroneState k3(this->get_increment({&k1, &k2}, {.09375, .28125}, current_dynamic_timestep), control);
    //printf("k1: %f\n", k1.d_lin_vel[2]);
    DDroneState k4(this->get_increment({&k1, &k2, &k3}, {.879380974, -3.277196177, 3.320892126}, current_dynamic_timestep), control);
    DDroneState k5(this->get_increment({&k1, &k2, &k3, &k4}, {2.032407407, -8.0, 7.173489279, -0.205896686}, current_dynamic_timestep), control);
    //printf("k1: %f\n", k1.d_lin_vel[2]);
    DDroneState k6(this->get_increment({&k1, &k2, &k3, &k4, &k5}, {-0.296296296, 2.0, -1.381676413, 0.45297271, -0.275}, current_dynamic_timestep), control);

    //printf("Here-------------------------\n");

    //get fifth order est for error correction
    DroneState est_5 = this->get_increment({&k1, &k3, &k4, &k5, &k6}, {0.118518519, 0.518986355, 0.50613149, -0.18, 0.03636363}, current_dynamic_timestep);

    //fourth order iterate state
    this->update({&k1, &k3, &k4, &k5}, {0.115740741, 0.548927875, 0.535722994, -0.2}, current_dynamic_timestep);

    //calculate the optimal step size
    current_dynamic_timestep = current_dynamic_timestep * powf(
        (DYNAMIC_INTEGRATION_ERROR_TOLERANCE) / (est_5.get_error(this)),
         .025);

    if(current_dynamic_timestep < previous_timestep){
        //revert to the previous state
        position = previous_position;
        rotation = previous_rotation;
        twist = previous_twist;

        //recalculate with the small timestep
        update_rk45(control);

    } else {
        stamp += current_dynamic_timestep * 1e6;
    }

    //printf("Error: %f Updated step: %f\n", est_5.get_error(this) * 1e6, current_dynamic_timestep* 1e10);

    if(std::isnan(k1.d_lin_vel[2])){
        int* quit = nullptr;
        int q = *quit;
    }


}

void DroneState::set_control(V8f control){
    current_control = control;
}


DDroneState::DDroneState(DroneState state, V8f* control) : DDroneState(&state, control){
    //secondary constructor...
}

DDroneState::DDroneState(DroneState* state, V8f* control){
    //get the change in velocity based on the current state and the control inputs

    //get the body frame forces due to the control inputs
    V6f thrust_effect = state->model->thruster_effect * *control;

    //printf("Current Twist: %f %f %f\n", state->twist[0], state->twist[1], state->twist[2]);

    //get the body frame forces and torques due to drag
    V6f drag_effect = -state->model->first_order_drag_cofs.cwiseProduct(state->twist);// + -state->model->second_order_drag_cofs.cwiseProduct(state->twist.cwiseProduct(state->twist.cwiseAbs()));

    //since the angular velocities will be minial, it is assumed that any effects caused by the angular drags being offset from the com is neglidable
    V3f offset_drag_effect = (state->model->cod - state->model->com).cross(drag_effect.block<3,1>(0,0));
    drag_effect(3) += offset_drag_effect(0);
    drag_effect(4) += offset_drag_effect(1);
    drag_effect(5) += offset_drag_effect(2);

    //get the linear accelerations in world frame...
    d_lin_vel = (state->rotation * (thrust_effect.block<3,1>(0,0) + drag_effect.block<3,1>(0,0)) / SBD_MASS + state->model->gravitational_acceleration);
    //printf("d_vel: %f %f %f \n", d_lin_vel[0], d_lin_vel[1], d_lin_vel[2]);
    //printf("thrust: %f %f %f \n", thrust_effect[0], thrust_effect[1], thrust_effect[2]);
    //printf("drag: %f %f %f \n", drag_effect[0], drag_effect[1], drag_effect[2]);


    //get the effect caused by prop drag
    //NOTE this torque is not effected by the COM being offset from center of the drone... it balanaces out... (Pretty easy to conceptualize)
    V3f prop_drag_effect(0.0, 0.0, state->model->convert_forces_to_w(*control).dot(state->model->prop_drag));

    //centripical effects
    //for the sake of the centripital effects, making the assumption that the drone is a homogenous disk, therefore the com is the center of the disk
    //this gets nasty if the above assumption does not hold
    V3f centripical_effects(
        (state->model->rotational_interia(1,1) - state->model->rotational_interia(2,2)) / state->model->rotational_interia(0,0) * state->twist(4) * state->twist(5),
        (state->model->rotational_interia(2,2) - state->model->rotational_interia(0,0)) / state->model->rotational_interia(1,1) * state->twist(3) * state->twist(4),
        (state->model->rotational_interia(0,0) - state->model->rotational_interia(1,1)) / state->model->rotational_interia(2,2) * state->twist(3) * state->twist(5)
    );

    //"body frame rotaitons"
    d_ang_vel = (thrust_effect.block<3,1>(3,0) + drag_effect.block<3,1>(3,0) + prop_drag_effect).cwiseQuotient(V3f(state->model->rotational_interia(0,0), state->model->rotational_interia(1,1), state->model->rotational_interia(2,2))) - centripical_effects;

    //printf("d_ang_vel: %f %f %f \n", d_ang_vel[0], d_ang_vel[1], d_ang_vel[2]);

    //integrate the velocity to get the positional change
    d_pos = state->rotation * state->twist.block<3,1>(0,0);


    //integrate the chnage in rotation
    d_rot = state->twist.block<3,1>(3,0);
    //printf("Twist %f %f %f\n", d_pos[0], d_pos[1], d_pos[2]);
}

DDroneState::DDroneState(V3f dlv_1, V3f dlv_2, V3f dav_1, V3f dav_2, V3f dp_1, V3f dp_2, V3f dr_1, V3f dr_2){
    d_lin_vel = dlv_1 + dlv_2;
    d_ang_vel = dav_1 + dav_2;
    d_pos = dp_1 + dp_2;
    d_rot = dr_1 + dr_2;
}




