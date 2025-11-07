#include "imu.h"

IMU::IMU(){
    //do things to start the imu
    imu_cov <<
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
}

void IMU::ackDataReady(){
    data_ready = true;
    data_ready_time = get_absolute_time();
}

void IMU::pollIMU(){
    //poll the imu for data if there is data ready
    if(data_ready){
        v6d current_measurement;

        //replace this with acutally getting the measurement
        current_measurement << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        //add to queue
        outstanding_measurements.push_back({data_ready_time, current_measurement});
    }

    data_ready = false;
}

std::vector<std::pair<uint64_t, v6d>> IMU::getOutstandingMeasurements(){
    //check if the imu needs polled before returning
    if(POLL_BEFORE_GET && data_ready){
         pollIMU();
    }

    //copy and clear outstanding measurements
    std::vector<std::pair<uint64_t, v6d>> outstanding_measurements_cp = outstanding_measurements;
    outstanding_measurements.clear();

    return outstanding_measurements_cp;
}