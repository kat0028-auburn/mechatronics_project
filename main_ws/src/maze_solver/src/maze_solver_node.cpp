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
    int steps_since_valid;
    int steps_since_correction;
    bool recv;

    void sonarCallback(const hardware_serial_interface::SonarArray::ConstPtr &msg);

    void turnLeft();
    void turnRight();
    void goForward(const int &cmd = 20);
    void turnAround();
    void shiftLeft(const double &val);
    void shiftRight(const double &val);
    void checkCalibration(const double &left_range, const double &right_range, const double &front_range);
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
    this->steps_since_valid = 0;
    this->steps_since_correction = 0;
    this->recv = false;
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
        int cmd = (msg->sonar_front  - 10) * 20;
        if (cmd < 0)
        {
            cmd = 20;
        }
        else if(cmd > 500)
        {
            cmd = 500;
        }
        
        goForward(cmd);
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

    checkCalibration(msg->sonar_left, msg->sonar_right, msg->sonar_front);
    
    if (msg->sonar_front > (front_tolerance + 10) && msg->sonar_right < 10)
    {
        shiftLeft((10-msg->sonar_right) * 50);
    }
    if (msg->sonar_front > (front_tolerance + 10) && msg->sonar_left < 10)
    {
        shiftRight((10-msg->sonar_left) * 50);
    }

    recv = true;
}

void MazeSolverNode::goForward(const int& cmd)
{
    hardware_serial_interface::StepperArray stepper_msg;
    stepper_msg.mode = 1;
    stepper_msg.steps = cmd;
    stepper_msg.header.stamp = ros::Time::now();
    motor_pub.publish(stepper_msg);

    steps_since_valid += stepper_msg.steps;
    steps_since_correction += stepper_msg.steps;
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
        stepper_msg.steps = turn_steps*2;
        stepper_msg.header.stamp = ros::Time::now();
        motor_pub.publish(stepper_msg);
        turn_cooldown = 3;

        recv = false;
        while (!recv)
        {
            ros::spinOnce();
        }
        
        stepper_msg.mode = 1;
        stepper_msg.steps = 1000;
        motor_pub.publish(stepper_msg);

        recv = false;
        while (!recv)
        {
            ros::spinOnce();
        }
        stepper_msg.mode = 7;
        motor_pub.publish(stepper_msg);
    }
    --turn_cooldown;
}

void MazeSolverNode::shiftLeft(const double &val)
{
    hardware_serial_interface::StepperArray stepper_msg;
    stepper_msg.mode = 5;
    stepper_msg.steps = val;
    stepper_msg.header.stamp = ros::Time::now();
    motor_pub.publish(stepper_msg);

    steps_since_valid += val;
    steps_since_correction += val;
}

void MazeSolverNode::shiftRight(const double &val)
{
    hardware_serial_interface::StepperArray stepper_msg;
    stepper_msg.mode = 6;
    stepper_msg.steps = val;
    stepper_msg.header.stamp = ros::Time::now();
    motor_pub.publish(stepper_msg);
    
    steps_since_valid += val;
    steps_since_correction += val;
}

void MazeSolverNode::checkCalibration(const double &left_range, const double &right_range, const double &range_front)
{
    bool heading_check = false;
    bool lateral_check = false;

    int total = left_range + right_range;
    int differnece = abs(left_range - right_range);
    
    if (total > 30 && total < 40)
    {
        heading_check = true;
        std::cout<< "HEADING TOLERANCE" << std::endl;
    }
    else 
    {
        heading_check = false;
    }

    if (differnece > 5)
    {
        lateral_check = true;
        std::cout << "LATERAL TOLERANCE" << std::endl;
    }
    else
    {
        lateral_check = false;
    }

    if (total < 50)
    {
        std::cout<<"Valid: "<<steps_since_valid<<", "<<steps_since_correction<<std::endl;
        if ((lateral_check || heading_check) && (steps_since_valid > 2000 && steps_since_correction > 4000 && range_front > 40) && (left_range <= 15 || right_range <= 15))
        {
            std::cout<<"CALIBRATE"<<std::endl;
            steps_since_correction = 0;
            steps_since_valid = 0;
            hardware_serial_interface::StepperArray msg;
            msg.mode = 7;
            msg.header.stamp = ros::Time::now();
            for (int i = 0; i < 3; i++){motor_pub.publish(msg);}
        }
        else if (!(left_range <= 15 || right_range <= 15))
        {
            std::cout<<"Calibration Blocked: Not Near Wall"<<std::endl;
        }
        else if (!(range_front > 40))
        {
            std::cout<<"Calibration Block: Too Close to front"<<std::endl;
        }
    }   
    else 
    {
        steps_since_valid = 0;
    }
    //hardware_serial_interface::StepperArray stepper_msg;
    //stepper_msg.mode = 7;
    //motor_pub.publish(stepper_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_solver");
    MazeSolverNode node;
    ros::spin();

    return 0;
}