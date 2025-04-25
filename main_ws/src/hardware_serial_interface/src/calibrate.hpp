#ifndef CALIBRATE_HPP
#define CALIBRATE_HPP

#include <hardware_serial_interface/StepperArray.h>
#include <vector>

class Calibrate
{
    public:
    Calibrate(const int &turn_steps);
    ~Calibrate();

    bool getCalibration();
    void setCalibration(bool start = true);
    void setMeasurements(const double& left_range, const double& right_range);
    hardware_serial_interface::StepperArray getMotorCmd();

    private:
    int steps_from_start;
    std::vector<std::pair<int, double>> data;
    std::vector<std::pair<int, double>> smoothed_data;
    std::pair<int, double> min;

    double left_range;
    double right_range;

    int step_size;
    int search_window;
    bool calibrating;
    bool first_pass;
    bool second_pass;
    bool solved;
    bool heading_calibrated;
    bool lateral_calibrated;

    int lateral_cal_mode;
    int lateral_phase;
    int turn_steps;
    int lateral_error;

    int findMinPoint();
    void prepareData();
    double average(std::vector<double> nums);
    bool checkLateralCalibration();
    void lateralCalibration();
};

#endif