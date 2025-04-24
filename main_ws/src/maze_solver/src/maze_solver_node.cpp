#include <ros/ros.h>
#include <hardware_serial_interface/StepperArray.h>
#include <hardware_serial_interface/SonarArray.h>

class MazeSolverNode
{
    public: 
    MazeSolverNode();
    ~MazeSolverNode();

    private:
    ros::NodeHandle n;
    ros::Subscriber sonar_sub;
    ros::Publisher motor_pub;

    int turn_steps;

    void sonarCallback(const hardware_serial_interface::SonarArray::ConstPtr &msg);
};

MazeSolverNode::MazeSolverNode()
{
    this->n = ros::NodeHandle("~");
    
    std::string sonar_topic;
    std::string motor_topic;

    this->n.param<std::string>("sonar_topic", sonar_topic, "sonar");
    this->n.param<std::string>("motor_topic", motor_topic, "motor");
    this->n.param<int>("turn_steps", turn_steps, 1150);

    this->sonar_sub = this->n.subscribe(sonar_topic, 1, &MazeSolverNode::sonarCallback, this);
    this->motor_pub = this->n.advertise<hardware_serial_interface::StepperArray>(motor_topic, 1);
}

MazeSolverNode::~MazeSolverNode()
{
    // Do Nothing
}

void MazeSolverNode::sonarCallback(const hardware_serial_interface::SonarArray::ConstPtr &msg)
{
    std::cout << msg->sonar_front << ", " << msg->sonar_left << ", " << msg->sonar_right << std::endl;
    hardware_serial_interface::StepperArray stepper_msg;

    if (msg->sonar_front > 20)
    {
        stepper_msg.mode = 1;
        stepper_msg.steps = 20;
    }
    else if (msg->sonar_left > 20)
    {
        stepper_msg.mode = 3;
        stepper_msg.steps = turn_steps;
    }
    else if(msg->sonar_right > 20)
    {
        stepper_msg.mode = 4;
        stepper_msg.steps = turn_steps;
    }

    motor_pub.publish(stepper_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_solver");
    MazeSolverNode node;
    ros::spin();

    return 0;
}