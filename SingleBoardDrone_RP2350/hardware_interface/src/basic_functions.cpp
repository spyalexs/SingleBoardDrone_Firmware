#include "basic_functions.hpp"


//@brief get the time since boot in microseconds
//@return time in microseconds
uint64_t get_hardware_time(){
    return time_us_64();
}