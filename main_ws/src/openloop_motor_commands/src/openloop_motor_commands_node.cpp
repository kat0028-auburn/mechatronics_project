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

    void callback(const hardware_serial_interface::SonarArray::ConstPtr &msg);
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
    send_message = true;
    std::cout<<"received"<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openloop_motor_commands");
    OpenloopMotorCommandsNode node;
    ros::Rate loop_rate(10);
    while (!node.send_message)
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    std::cout<<"here"<<std::endl;
    hardware_serial_interface::StepperArray msg;
    msg.header.stamp = ros::Time::now();
    
    msg.steps = 1150 * 8;
    msg.mode = 4;
    node.publishMessage(msg);

    return 0;
}