#include <ros/ros.h>
#include <hardware_serial_interface/SonarArray.h>
#include <hardware_serial_interface/StepperArray.h>

class OpenloopMotorCommandsNode
{
    public: 
    OpenloopMotorCommandsNode();
    ~OpenloopMotorCommandsNode();
    
    bool send_message;
    void publishMessage(hardware_serial_interface::StepperArray msg);

    private: 
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    bool calibrate;

    void callback(const hardware_serial_interface::SonarArray::ConstPtr &msg);
    bool checkCalibration(const int &left_range, const int& right_range);
};

OpenloopMotorCommandsNode::OpenloopMotorCommandsNode()
{
    std::string sub_topic;
    std::string pub_topic; 

    this->n = ros::NodeHandle("~");
    this->n.param<std::string>("sub_topic", sub_topic, "sub");
    this->n.param<std::string>("pub_topic", pub_topic, "pub");

    this->pub = n.advertise<hardware_serial_interface::StepperArray>(pub_topic, 1);
    this->sub = n.subscribe(sub_topic, 1, &OpenloopMotorCommandsNode::callback, this);

    this->send_message = false;
    this->calibrate = false;
}

OpenloopMotorCommandsNode::~OpenloopMotorCommandsNode()
{

}

void OpenloopMotorCommandsNode::publishMessage(hardware_serial_interface::StepperArray msg)
{
    this->pub.publish(msg);
}

void OpenloopMotorCommandsNode::callback(const hardware_serial_interface::SonarArray::ConstPtr &msg)
{
    // Run Calibration Checks 
    if (checkCalibration(msg->sonar_left, msg->sonar_right))
    {
        std::cout<<"Run Calibration"<<std::endl;
    }
    else
    {
        std::cout<<"Calibration Good"<<std::endl;
    }
    send_message = true;
}

bool OpenloopMotorCommandsNode::checkCalibration(const int &left_range, const int &right_range)
{
    std::cout << "Total Range: "<<left_range+right_range<<std::endl;
    return ((left_range + right_range) < 40 && (left_range + right_range) > 30);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openloop_motor_commands");
    OpenloopMotorCommandsNode node;
    ros::Rate loop_rate(10);
    while (!node.send_message && ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    /*
    std::cout<<"here"<<std::endl;
    hardware_serial_interface::StepperArray msg;
    msg.header.stamp = ros::Time::now();
    
    msg.steps = 1150 * 8;
    msg.mode = 4;
    node.publishMessage(msg);*/
    hardware_serial_interface::StepperArray msg;
    msg.mode = 3;
    msg.steps = 1120 * 8;
    msg.steps = 20;

    node.publishMessage(msg);

    return 0;
}