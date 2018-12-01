#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// messages used for the absolute and proportional movement topics
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

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
    ros::Publisher i2cpwm_pub_;
};

TeleopTurtle::TeleopTurtle():
    linear_(1), angular_(2)
{
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_linear", a_scale_, a_scale_);
    nh_.param("scale_angular", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    i2cpwm_pub_ = nh_.advertise<i2cpwm_board::ServoArray>("servos_proportional", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
        "joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linear_];
    vel_pub_.publish(twist);

    i2cpwm_board::Servo servo1;
    servo1.servo = 1;
    servo1.value = l_scale_ * joy->axes[linear_];

    i2cpwm_board::Servo servo2;
    servo2.servo = 2;
    servo2.value = a_scale_ * joy->axes[angular_];

    i2cpwm_board::ServoArray servo_array;
    servo_array.servos = {servo1, servo2};
    i2cpwm_pub_.publish(servo_array);
}

// rosservice call /config_drive_mode "{mode: ackerman, rpm: 56.0, radius: 0.038, track: 0.18, scale: 1.0, servos: [{servo: 1, position: 1}, {servo: 2, position: 1}]}"

// rosservice call /config_servos "servos: [{servo: 1, center: 333, range: 100, direction: 1}, {servo: 2, center: 336, range: 108, direction: -1}]"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_turtle");
    TeleopTurtle teleop_turtle;

    ros::spin();
}
