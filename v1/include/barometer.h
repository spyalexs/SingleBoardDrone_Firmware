#pragma once

#include <Eigen/Dense>
#include <vector>

#include "pico/time.h"

#include "physicalsingleboarddrone.h"


//poll the imu if data is ready before getting the outstanding measurements
#define POLL_BEFORE_GET true

class Barometer{
    public:
        Barometer();

        void ackDataReady();
        void pollBarometer();

        std::vector<std::pair<uint64_t, float>> getOutstandingMeasurements();

        bool data_ready = true;
        absolute_time_t data_ready_time;

        // queue of measurements that havent been intaken by the ekf yet
        std::vector<std::pair<uint64_t, float>> outstanding_measurements;

        float barometer_variance;
};
