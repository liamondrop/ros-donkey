#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "i2cpwm_board/ServosConfig.h"
#include "i2cpwm_board/ServoArray.h"
#include "i2cpwm_board/ServoConfig.h"

class TeleopTurtle
{
public:
    TeleopTurtle();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

TeleopTurtle::TeleopTurtle():
    linear_(1), angular_(2)
{
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_linear", a_scale_, a_scale_);
    nh_.param("scale_angular", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
        "joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linear_];
    vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_turtle");
    i2cpwm_board::ServosConfig srv;
    i2cpwm_board::ServoArray servos;
    i2cpwm_board::ServoConfig servo1;
    servo1.servo = 1;
    servo1.center = 333;
    servo1.range = 100;
    servo1.direction = -1;
    servos.push_back(servo1);

    i2cpwm_board::ServoConfig servo2;
    servo2.servo = 2;
    servo2.center = 336;
    servo2.range = 108;
    servo2.direction = 1;
    servos.push_back(servo2);

    ROS_INFO(servos);
    TeleopTurtle teleop_turtle;

    ros::spin();
}
