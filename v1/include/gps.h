#pragma once

#include <vector>
#include "Eigen/Dense"

#include "physicalsingleboarddrone.h"


class Gps{

    Gps();

    std::vector<std::pair<uint64_t, v3d>> outstanding_rmc_measurements;

    void ackDataReady();
    void pollGPS();

    v2d convert_lat_long(float lat, float lon);

    std::vector<std::pair<uint64_t, v7d>> getOutstandingMeasurements();

    bool data_ready = true;
    absolute_time_t data_ready_time;
};