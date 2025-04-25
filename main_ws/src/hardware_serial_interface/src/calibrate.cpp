#include "calibrate.hpp"

Calibrate::Calibrate()
{
    calibrating = false;
    first_pass = false;
    int steps_from_start = 0;
    data.clear();
    search_window = 400;
    step_size = 10;
}

Calibrate::~Calibrate()
{
    // Do Nothing
}

bool Calibrate::getCalibration()
{
    return (!this->calibrating);
}

void Calibrate::setCalibration(bool start)
{
    if (start)
    {
        this->calibrating = true;
        this->data.clear();
        this->steps_from_start = 0;
        this->first_pass = false;
    }
}

void Calibrate::setMeasurements(const double& left_range, const double& right_range)
{
    double total = left_range + right_range;
    std::pair<int, int> data_point; 
    data_point.first = steps_from_start;
    data_point.second = total;
    data.push_back(data_point);
}

hardware_serial_interface::StepperArray Calibrate::getMotorCmd()
{
    hardware_serial_interface::StepperArray msg;

    //std::cout<<steps_from_start << ", " << search_window<<std::endl;
    if (!first_pass)
    {
        msg.mode = 3;
        if (steps_from_start > search_window)
        {
            msg.mode = 4;
            msg.steps = steps_from_start;
            first_pass = true;
            steps_from_start = 0;
        }    
        else
        {
            msg.steps = step_size;
            steps_from_start += step_size;
        }
    }
    else
    {
        msg.mode = 4;
        if (steps_from_start < -1*search_window)
        {
            findMinPoint();
            //std::cout<<"Finished Calibration"<<std::endl;
            // Do something with this point
            for (std::pair<int, int> i : data)
            {
                //std::cout<<i.first << ", "<< i.second<<std::endl;
            }
        }
        else
        {
            msg.steps = step_size;
            steps_from_start -= step_size;
        }
    }

    return msg;
}

int Calibrate::findMinPoint()
{
    return 0;
}