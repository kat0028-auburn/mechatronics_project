#ifndef CALIBRATE_HPP
#define CALIBRATE_HPP

#include <hardware_serial_interface/StepperArray.h>
#include <vector>

class Calibrate
{
    public:
    Calibrate();
    ~Calibrate();

    bool getCalibration();
    void setCalibration(bool start = true);
    void setMeasurements(const double& left_range, const double& right_range);
    hardware_serial_interface::StepperArray getMotorCmd();

    private:
    int steps_from_start;
    std::vector<std::pair<int, int>> data;
    int step_size;
    int search_window;
    bool calibrating;
    bool first_pass;

    int findMinPoint();
};

#endif