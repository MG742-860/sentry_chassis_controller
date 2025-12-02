#ifndef _TELEOP_TWIST_KEYBOARD_HPP_
#define _TELEOP_TWIST_KEYBOARD_HPP_
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>

struct gotten_key
{
    int forward = 'w';
    int backward = 's';
    int left = 'a';
    int right = 'd';
    int forward_left = 'q';
    int forward_right = 'e';
    int backward_left = 'z';
    int backward_right = 'c';
    int stop = 'x';
    int increase_linear = 'u';
    int decrease_linear = 'j';
    int increase_angular = 'i';
    int decrease_angular = 'k';
    int left_rotate = 'f';
    int right_rotate = 'g';
    bool valid = false;
};

class TeleopTwistKeyboard
{
public:
    TeleopTwistKeyboard();
    ~TeleopTwistKeyboard();
    void printInstructions();
    void publishTwist(double linear_x, double linear_y, double angular_z);
    void run();
    int getKey();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher twist_pub_;
    int pub_rate_;
    geometry_msgs::Twist twist_msg_;
    std::string sub_topic_;
    double initial_linear_;
    double initial_angular_;
    double linear_step_;
    double angular_step_;
    struct termios cooked_, raw_;
    gotten_key key_bindings_;
};
#endif  //_TELEOP_TEIST_KEYBOARD_HPP_