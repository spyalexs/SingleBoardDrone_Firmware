#include "barometer.h"

Barometer::Barometer(){
    //do things to start the imu
    barometer_variance = 1.0; 
    
}

void Barometer::ackDataReady(){
    data_ready = true;
    data_ready_time = get_absolute_time();
}

void Barometer::pollBarometer(){
    //poll the imu for data if there is data ready
    if(data_ready){
        //replace this with acutally getting the measurement
        float current_measurement  = 0.0;

        //add to queue
        outstanding_measurements.push_back({data_ready_time, current_measurement});
    }

    data_ready = false;
}

std::vector<std::pair<uint64_t, float>> Barometer::getOutstandingMeasurements(){
    //check if the imu needs polled before returning
    if(POLL_BEFORE_GET && data_ready){
        pollBarometer();
    }

    //copy and clear outstanding measurements
    std::vector<std::pair<uint64_t, float>> outstanding_measurements_cp = outstanding_measurements;
    outstanding_measurements.clear();

    return outstanding_measurements_cp;
}