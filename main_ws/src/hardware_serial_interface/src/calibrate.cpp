#include "calibrate.hpp"

Calibrate::Calibrate()
{
    calibrating = false;
    first_pass = false;
    second_pass = false;
    solved = false;
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
        this->second_pass = false;
        this->solved = false;
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
    else if(!solved)
    {
        msg.mode = 3;
        if (steps_from_start >= 0)
        {
            // Calibration            
            prepareData();
            solved = true;
        }
        else
        {
            msg.steps = step_size;
            steps_from_start += step_size;
        }
    }
    else
    {
        if (steps_from_start == min.first)
        {
            this->calibrating = false;
        }
        else
        {
            msg.steps = step_size;

            if (min.first > 0)
            {
                msg.mode = 3;
                steps_from_start += step_size;
            }
            else
            {
                msg.mode = 4;
                steps_from_start -= step_size;
            }
            std::cout<<"Centering: " << steps_from_start << ", " << min.first << ", " << msg.steps << std::endl;
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
    std::vector<std::pair<int, double>> unsorted = data;
    data.clear();

    for (int i = -search_window; i < search_window; i += 10)
    {
        std::pair<int, double> point;
        point.first = i;
        
        std::vector<double> vals;
        for (std::pair<int, double> item : unsorted)
        {
            if (item.first == i)
            {
                vals.push_back(item.second);
            }
        }
        point.second = average(vals);
        data.push_back(point);
    }

    smoothed_data.clear();
    for (size_t i = 2; i < data.size()-2; i++)
    {
        std::vector<double> vals;
        for (int j = -2; j < 3; j++)
        {
            vals.push_back(data.at(i - j).second);
        }
        std::cout<<vals.size();
        std::pair<int, double> point;
        point.first = data.at(i).first;
        point.second = average(vals);
        smoothed_data.push_back(point);
    }
    
    for (std::pair<int, double> item : smoothed_data)
    {
        std::cout<<item.first<<", "<<item.second<<std::endl;
    }

    min = smoothed_data.at(0);
    for (std::pair<int, double> item : smoothed_data)
    {
        if (item.second < min.second)
        {
            min = item;
        }
    }

    std::cout << "Minimum: " << min.first << ", " << min.second<<std::endl;
}

double Calibrate::average(std::vector<double> nums)
{
    double total = 0;
    for (double i : nums)
    {
        total += i;
    }

    return (total/nums.size());
}