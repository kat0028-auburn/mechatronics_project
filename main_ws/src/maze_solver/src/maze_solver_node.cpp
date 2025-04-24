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
    int turn_cooldown;
    int front_tolerance;
    int side_tolerance;

    void sonarCallback(const hardware_serial_interface::SonarArray::ConstPtr &msg);

    void turnLeft();
    void turnRight();
    void goForward();
    void turnAround();
};

MazeSolverNode::MazeSolverNode()
{
    this->n = ros::NodeHandle("~");
    
    std::string sonar_topic;
    std::string motor_topic;

    this->n.param<std::string>("sonar_topic", sonar_topic, "sonar");
    this->n.param<std::string>("motor_topic", motor_topic, "motor");
    this->n.param<int>("turn_steps", this->turn_steps, 1150);
    this->n.param<int>("front_tolerance", this->front_tolerance, 20);
    this->n.param<int>("side_tolerance", this->side_tolerance, 40);

    this->sonar_sub = this->n.subscribe(sonar_topic, 1, &MazeSolverNode::sonarCallback, this);
    this->motor_pub = this->n.advertise<hardware_serial_interface::StepperArray>(motor_topic, 1);

    this->turn_cooldown = 0;
}

MazeSolverNode::~MazeSolverNode()
{
    // Do Nothing
}

void MazeSolverNode::sonarCallback(const hardware_serial_interface::SonarArray::ConstPtr &msg)
{
    std::cout << msg->sonar_front << ", " << msg->sonar_left << ", " << msg->sonar_right << std::endl;

    if (msg->sonar_front > front_tolerance)
    {
        goForward();
    }
    else if (msg->sonar_left > side_tolerance)
    {
        turnLeft();
    }
    else if(msg->sonar_right > side_tolerance)
    {
        turnRight();
    }
    else 
    {
        turnAround();
    }
}

void MazeSolverNode::goForward()
{
    hardware_serial_interface::StepperArray stepper_msg;
    stepper_msg.mode = 1;
    stepper_msg.steps = 50;
    stepper_msg.header.stamp = ros::Time::now();
    motor_pub.publish(stepper_msg);
}

void MazeSolverNode::turnLeft()
{
    if (!turn_cooldown)
    {
        hardware_serial_interface::StepperArray stepper_msg;
        stepper_msg.mode = 3;
        stepper_msg.steps = turn_steps;
        stepper_msg.header.stamp = ros::Time::now();
        motor_pub.publish(stepper_msg);
        turn_cooldown = 3;
    }
    --turn_cooldown;
}

void MazeSolverNode::turnRight()
{
    if (!turn_cooldown)
    {
        hardware_serial_interface::StepperArray stepper_msg;
        stepper_msg.mode = 4;
        stepper_msg.steps = turn_steps;
        stepper_msg.header.stamp = ros::Time::now();
        motor_pub.publish(stepper_msg);
        turn_cooldown = 3;
    }
    --turn_cooldown;
}

void MazeSolverNode::turnAround()
{
    if (!turn_cooldown)
    {
        hardware_serial_interface::StepperArray stepper_msg;
        stepper_msg.mode = 3;
        stepper_msg.steps = turn_steps;
        stepper_msg.header.stamp = ros::Time::now();
        motor_pub.publish(stepper_msg);
        turn_cooldown = 3;
    }
    --turn_cooldown;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_solver");
    MazeSolverNode node;
    ros::spin();

    return 0;
}