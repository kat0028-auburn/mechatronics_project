#include "calibrate.hpp"

Calibrate::Calibrate()
{
    calibrating = false;
    first_pass = false;
    second_pass = false;
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

    if (!first_pass)
    {
        msg.mode = 3;
        if (steps_from_start >= search_window)
        {
            first_pass = true;
        }    
        else
        {
            msg.steps = step_size;
            steps_from_start += step_size;
        }
    }
    else if(!second_pass)
    {
        msg.mode = 4;
        if (steps_from_start <= -1*search_window)
        {
            second_pass = true;
        }
        else
        {
            msg.steps = step_size;
            steps_from_start -= step_size;
        }
    }
    else
    {
        msg.mode = 3;
        if (steps_from_start >= 0)
        {
            // Calibration
            this->calibrating = false;
            
            prepareData();
            for (std::pair<int, int> i : data)
            {
                std::cout<<i.first<<", "<<i.second<<std::endl;
            }
        }
        else
        {
            msg.steps = step_size;
            steps_from_start += step_size;
        }
    }

    return msg;
}

int Calibrate::findMinPoint()
{
    return 0;
}

void Calibrate::prepareData()
{
    std::vector<std::pair<int, int>> unsorted = data;
    data.clear();

    for (int i = -search_window; i < search_window; ++i)
    {
        std::pair<int, int> point;
        point.first = i;
        
        std::vector<int> vals;
        for (std::pair<int, int> item : unsorted)
        {
            if (item.first == i)
            {
                vals.push_back(item.second);
            }
        }
        point.second = average(vals);
        data.push_back(point);
    }
}

int Calibrate::average(std::vector<int> nums)
{
    int total = 0;
    for (int i : nums)
    {
        total += i;
    }
    double a = round(total/nums.size());

    return (int)a;
}