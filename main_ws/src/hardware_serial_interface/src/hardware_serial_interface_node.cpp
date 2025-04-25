#include <ros/ros.h>
#include <hardware_serial_interface/StepperArray.h>
#include <hardware_serial_interface/SonarArray.h>
#include <serial/serial.h>
#include <sstream>
#include <boost/algorithm/string.hpp>

#include "calibrate.hpp"

class HardwareSerialInterfaceNode
{
    public:
    HardwareSerialInterfaceNode();
    ~HardwareSerialInterfaceNode();

    void checkPort();
    void calibrate();

    private:
    ros::NodeHandle n;
    ros::Publisher sonar_array_pub;
    ros::Subscriber stepper_sub;

    serial::Serial* port;
    std::string motor_cmd_msg;
    bool pause_serial_writer;
    bool calibration_good;
    Calibrate cal_tool;

    void stepperCallback(const hardware_serial_interface::StepperArray::ConstPtr &msg);
};

HardwareSerialInterfaceNode::HardwareSerialInterfaceNode()
{
    this->n = ros::NodeHandle("~");

    std::string sonar_topic;
    std::string stepper_topic;
    std::string portname;
    int baudrate;
    pause_serial_writer = true;
    calibration_good = true;

    motor_cmd_msg = "0";

    this->n.param<std::string>("sonar_topic", sonar_topic, "sonar");
    this->n.param<std::string>("stepper_topic", stepper_topic, "stepper_cmd");
    this->n.param<std::string>("portname", portname, "/dev/ttyUSB0");
    this->n.param<int>("baudrate", baudrate, 115200);

    this->sonar_array_pub = this->n.advertise<hardware_serial_interface::SonarArray>(sonar_topic, 1);
    this->stepper_sub = this->n.subscribe(stepper_topic, 1, &HardwareSerialInterfaceNode::stepperCallback, this);

    this->port = new serial::Serial(portname, baudrate);
}

HardwareSerialInterfaceNode::~HardwareSerialInterfaceNode()
{
    delete this->port;
}

void HardwareSerialInterfaceNode::stepperCallback(const hardware_serial_interface::StepperArray::ConstPtr &msg)
{
    //motor_cmd_msg = std::to_string(msg->data);
    checkPort();
    std::cout<<"Main motor control"<<std::endl;
    if (msg->mode == 7)
    {
        calibration_good = false;
        calibrate();
    }
    else if (!pause_serial_writer && calibration_good)
    {
        std::cout<<"SHOULD NOT SEE THIS"<<std::endl;
        motor_cmd_msg = std::to_string(msg->mode) + "," + std::to_string(msg->steps);
        std::cout << motor_cmd_msg << std::endl;
        port->write(motor_cmd_msg);
        pause_serial_writer = true;
    }

    // Check Calibration Status
    
}

void HardwareSerialInterfaceNode::checkPort()
{
    std::string in_msg;
    if (port->available())
    {
        in_msg = this->port->readline();
        std::vector<std::string> result;
        boost::split(result, in_msg, boost::is_any_of(","));
        
        if (result.size() > 2)
        {    
            hardware_serial_interface::SonarArray msg;
            msg.header.stamp = ros::Time::now();
            msg.sonar_front = std::stof(result.at(2).c_str());
            msg.sonar_left = std::stof(result.at(1).c_str());
            msg.sonar_right = std::stof(result.at(0).c_str());
            this->sonar_array_pub.publish(msg);

            pause_serial_writer = false;
        }
    }
    //this->port->write(this->motor_cmd_msg);
}

void HardwareSerialInterfaceNode::calibrate()
{
    std::string in_msg;
    int left_range;
    int right_range;
    bool recv_message = false;
    
    cal_tool.setCalibration();
    while(!cal_tool.getCalibration())
    {
        if (port->available())
        {
            in_msg = this->port->readline();
            std::vector<std::string> result;
            boost::split(result, in_msg, boost::is_any_of(","));

            if (result.size() > 2)
            {    
                // msg.sonar_front = std::stof(result.at(2).c_str());
                left_range = std::stof(result.at(1).c_str());
                right_range = std::stof(result.at(0).c_str());
                recv_message = true;
            }
        }
        
        if (recv_message)
        {
            //std::cout << left_range << ", " << right_range<<std::endl;
            cal_tool.setMeasurements(left_range, right_range);
            hardware_serial_interface::StepperArray msg = cal_tool.getMotorCmd();
            std::cout<<msg.steps<<std::endl;
            motor_cmd_msg = std::to_string(msg.mode) + "," + std::to_string(msg.steps);
            port->write(motor_cmd_msg);
            recv_message = false;
        }
    }    

    calibration_good = !cal_tool.getCalibration();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_serial_interface");
    HardwareSerialInterfaceNode node;
    ros::Rate loop_rate(500);

    while(ros::ok())
    {
        node.checkPort();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}