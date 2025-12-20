#pragma once

//@brief Thruster poses
#define T1_POSE Eigen::Matrix<float, 7, 1>(0.0923, 0.0382, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T2_POSE Eigen::Matrix<float, 7, 1>(0.0382, 0.0923, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T3_POSE Eigen::Matrix<float, 7, 1>(-0.0382, 0.0923, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T4_POSE Eigen::Matrix<float, 7, 1>(-0.0923, 0.0382, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T5_POSE Eigen::Matrix<float, 7, 1>(-0.0923, -0.0382, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T6_POSE Eigen::Matrix<float, 7, 1>(-0.0382, -0.0923, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T7_POSE Eigen::Matrix<float, 7, 1>(0.0382, -0.0923, 0.01, 0.707, 0.0, -0.707, 0.0)
#define T8_POSE Eigen::Matrix<float, 7, 1>(0.0923, -0.0382, 0.01, 0.707, 0.0, -0.707, 0.0)

//@brief Thruster pose vector
#define THRUST_POSES {T1_POSE, T2_POSE, T3_POSE, T4_POSE, T5_POSE, T6_POSE, T7_POSE, T8_POSE}

//@brief The default unit vector for thrust directions
#define UNIT_THRUST Eigen::Matrix<float, 3, 1>(0.0, 0.0, 1.0)

//@brief Drone mass
#define SBD_MASS 0.12

//@brief Drone center of mass
#define SBD_COM Eigen::Matrix<float, 3, 1>(0.0, 0.0, 0.0)

//@brief thust constant for the drone...
#define SBD_THRUST_CONSTANT 0.1

//@brief 3D rotational interation matrix
#define SBD_ROT_INTERTIA Eigen::Matrix<float, 3, 3>(\
    0.01, 0.0, 0.0,\
    0.0, 0.01, 0.0,\
    0.0, 0.0, 0.01)

//@brief Drone center of drag
#define SBD_COD Eigen::Matrix<float, 3, 1>(0.0, 0.0, 0.0);

//@brief Drone first order drag coefficents
#define SBD_FIRST_DRAG Eigen::Matrix<float, 6, 1>(1.0, 1.0, 1.0, 0.1, 0.1, 0.1)

//@brief Drone second order drag coefficents
#define SBD_SECOND_DRAG Eigen::Matrix<float, 6, 1>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

//@brief The propellor drag coefficents, note the direction indicating the clockwise or counter clockwise rotation of the drone
#define SDB_PROP_DRAG Eigen::Matrix<float, 8, 1>(0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001, -0.001);

//@brief The gravitational vector used in this
#define GRAVITY_VECTOR Eigen::Matrix<float, 3, 1>(0.0, 0.0, 9.81)

//@brief rk45 integration error tolerance
#define DYNAMIC_INTEGRATION_ERROR_TOLERANCE 1e7
