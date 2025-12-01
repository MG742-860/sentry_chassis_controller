#include "../include/teleop_twist_keyboard.hpp"

// 辅助函数：读取字符参数
int readCharParam(ros::NodeHandle& nh, const std::string& param_name, int default_char) {
    // 尝试多种方式读取
    int result = default_char;
    
    // 方法1：直接读取整数
    if (nh.getParam(param_name, result)) {
        ROS_INFO("Successfully read integer parameter %s: %d (char: %c)", 
                 param_name.c_str(), result, (char)result);
        return result;
    }
    
    // 方法2：尝试读取字符串
    std::string str_value;
    if (nh.getParam(param_name, str_value)) {
        if (!str_value.empty()) {
            result = str_value[0];
            ROS_INFO("Read string parameter %s: %s -> char: %c (ASCII: %d)", 
                     param_name.c_str(), str_value.c_str(), (char)result, result);
            return result;
        }
    }
    
    ROS_WARN("Parameter %s not found, using default: %c", 
             param_name.c_str(), (char)default_char);
    return default_char;
}

TeleopTwistKeyboard::TeleopTwistKeyboard():nh_("~")
{
    //初始化数值
    twist_msg_.linear.x = 0.0;
    twist_msg_.linear.y = 0.0;
    twist_msg_.angular.z = 0.0;//只用到这三个分量
    initial_linear_ = 0.1;
    initial_angular_ = 0.01;
    linear_step_ = 0.1;
    angular_step_ = 0.01;
    pub_rate_ = 10; // 10 Hz

    // 初始化发布器
    nh_.param("topic", sub_topic_, std::string("cmd_vel"));
    nh_.param("publish_rate", pub_rate_, pub_rate_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(sub_topic_, pub_rate_);
    
    //初始化键盘控制参数
    key_bindings_.forward = 'w';
    key_bindings_.backward = 's';
    key_bindings_.left = 'a';
    key_bindings_.right = 'd'; 
    key_bindings_.forward_left = 'q';
    key_bindings_.forward_right = 'e';
    key_bindings_.backward_left = 'z';
    key_bindings_.backward_right = 'c';
    key_bindings_.stop = 'x';
    key_bindings_.increase_linear = 'u';
    key_bindings_.decrease_linear = 'j';
    key_bindings_.increase_angular = 'i';
    key_bindings_.decrease_angular = 'k';

    // 从参数服务器获取参数
    nh_.param("initial_linear_x", twist_msg_.linear.x, twist_msg_.linear.x);
    nh_.param("initial_linear_y", twist_msg_.linear.y, twist_msg_.linear.y);
    nh_.param("initial_angular_z", twist_msg_.angular.z, twist_msg_.angular.z);
    nh_.param("initial_linear", initial_linear_, initial_linear_);
    nh_.param("initial_angular", initial_angular_, initial_angular_);
    nh_.param("linear_step", linear_step_, linear_step_);
    nh_.param("angular_step", angular_step_, angular_step_);
    
    key_bindings_.forward = readCharParam(nh_, "key_bindings/forward", key_bindings_.forward);
    key_bindings_.backward = readCharParam(nh_, "key_bindings/backward", key_bindings_.backward);
    key_bindings_.left = readCharParam(nh_, "key_bindings/left", key_bindings_.left);
    key_bindings_.right = readCharParam(nh_, "key_bindings/right", key_bindings_.right);
    key_bindings_.forward_left = readCharParam(nh_, "key_bindings/forward_left", key_bindings_.forward_left);
    key_bindings_.forward_right = readCharParam(nh_, "key_bindings/forward_right", key_bindings_.forward_right);
    key_bindings_.backward_left = readCharParam(nh_, "key_bindings/backward_left", key_bindings_.backward_left);
    key_bindings_.backward_right = readCharParam(nh_, "key_bindings/backward_right", key_bindings_.backward_right);
    key_bindings_.stop = readCharParam(nh_, "key_bindings/stop", key_bindings_.stop);
    key_bindings_.increase_linear = readCharParam(nh_, "key_bindings/increase_linear", key_bindings_.increase_linear);
    key_bindings_.decrease_linear = readCharParam(nh_, "key_bindings/decrease_linear", key_bindings_.decrease_linear);
    key_bindings_.increase_angular = readCharParam(nh_, "key_bindings/increase_angular", key_bindings_.increase_angular);
    key_bindings_.decrease_angular = readCharParam(nh_, "key_bindings/decrease_angular", key_bindings_.decrease_angular);
    
    // 保存终端设置
    tcgetattr(STDIN_FILENO, &cooked_);
    memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &=~ (ICANON | ECHO);
    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    
    ROS_INFO("Teleop Twist Keyboard Started");
    printInstructions();
}

TeleopTwistKeyboard::~TeleopTwistKeyboard()
{
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked_);
    
    // 发布停止消息
    geometry_msgs::Twist twist;
    twist_pub_.publish(twist);
    ROS_INFO("Teleop Teist Keyboard Stopped");
}

void TeleopTwistKeyboard::printInstructions()
{
    std::cout << "Teleop Twist Keyboard Instructions:\n";
    std::cout << "Use "<< (char)(key_bindings_.forward) << ", " << (char)(key_bindings_.left) << ", " << (char)(key_bindings_.backward) << ", " << (char)(key_bindings_.right) << " to move the robot.\n";
    std::cout << "Use "<< (char)(key_bindings_.increase_linear) << ", " << (char)(key_bindings_.decrease_linear) << " to increase/decrease linear step.\n";
    std::cout << "Use "<< (char)(key_bindings_.increase_angular) << ", " << (char)(key_bindings_.decrease_angular) << " to increase/decrease angular step.\n";
    std::cout << "Press CTRL+C to quit." << std::endl;
}

int TeleopTwistKeyboard::getKey()
{
    int key = 0;
    
    // 设置非阻塞终端
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
    
    fd_set set;
    struct timeval timeout;
    
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms
    
    if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
        if (read(STDIN_FILENO, &key, 1) < 0) {
            perror("read():");
        }
    }
    
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked_);
    
    return key;
}

void TeleopTwistKeyboard::publishTwist(double linear_x, double linear_y, double angular_z)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linear_x;
    twist.linear.y = linear_y;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = angular_z;
    
    twist_pub_.publish(twist);
}

void TeleopTwistKeyboard::run()
{
    char key;
    bool quit = false;
    
    ROS_INFO("Ready for keyboard commands...");

    while(ros::ok() && !quit)
    {
        key = getKey();
        
        //每次发布前将数值清零
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.angular.z = 0.0;

        if (key != 0) 
        {
            if(key == key_bindings_.forward)
            {
                twist_msg_.linear.x = initial_linear_;
            }
            else if(key == key_bindings_.backward)
            {
                twist_msg_.linear.x = -initial_linear_;
            }
            else if(key == key_bindings_.left)
            {
                twist_msg_.linear.y = initial_linear_;
                twist_msg_.angular.z = initial_angular_;
            }
            else if(key == key_bindings_.right)
            {
                twist_msg_.linear.y = -initial_linear_;
                twist_msg_.angular.z = -initial_angular_;
            }
            else if(key == key_bindings_.stop)
            {
                // all zero
            }
            else if(key == key_bindings_.forward_left)
            {
                twist_msg_.linear.x = initial_linear_;
                twist_msg_.linear.y = initial_linear_;
            }
            else if(key == key_bindings_.forward_right)
            {
                twist_msg_.linear.x = initial_linear_;
                twist_msg_.linear.y = -initial_linear_;
            }
            else if(key == key_bindings_.backward_left)
            {
                twist_msg_.linear.x = -initial_linear_;
                twist_msg_.linear.y = initial_linear_;
            }
            else if(key == key_bindings_.backward_right)
            {
                twist_msg_.linear.x = -initial_linear_;
                twist_msg_.linear.y = -initial_linear_;
            }
            else if(key == key_bindings_.increase_linear)
            {
                initial_linear_ += linear_step_;
                printf("\rLinear step increased to: %.2f", initial_linear_);
            }
            else if(key == key_bindings_.decrease_linear)
            {
                initial_linear_ -= linear_step_;
                if (initial_linear_ < 0.1) initial_linear_ = 0.1;
                printf("\rLinear step decreased to: %.2f", initial_linear_);
            }
            else if(key == key_bindings_.increase_angular)
            {
                initial_angular_ += angular_step_;
                printf("\rAngular step increased to: %.2f", initial_angular_);
            }
            else if(key == key_bindings_.decrease_angular)
            {
                initial_angular_ -= angular_step_;
                if (initial_angular_ < 0.01) initial_angular_ = 0.01;
                printf("\rAngular step decreased to: %.2f", initial_angular_);
            }
            else if(key == 3) // CTRL+C
            {
                quit = true;
                std::cout << "\nExiting..." << std::endl;
            }  
            else
            {
                printf("\rUnknown command: %c", key);
            }
            // switch(key) 
            // {
            //     case key_bindings_.forward:
            //         twist_msg_.linear.x = linear_step_;
            //         twist_msg_.linear.y = 0.0;
            //         twist_msg_.angular.z = 0.0;
            //         break;
            //     case key_bindings_.backward:
            //         twist_msg_.linear.x = -linear_step_;
            //         twist_msg_.linear.y = 0.0;
            //         twist_msg_.angular.z = 0.0;
            //         break;
            //     case key_bindings_.left:
            //         twist_msg_.linear.x = 0.0;
            //         twist_msg_.linear.y = linear_step_;
            //         twist_msg_.angular.z = angular_step_;
            //         break;
            //     case key_bindings_.right:
            //         twist_msg_.linear.x = 0.0;
            //         twist_msg_.linear.y = -linear_step_;
            //         twist_msg_.angular.z = -angular_step_;
            //         break;     
            //     case key_bindings_.stop:
            //         twist_msg_.linear.x = 0.0;
            //         twist_msg_.linear.y = 0.0;
            //         twist_msg_.angular.z = 0.0;
            //         break; 
            //     case key_bindings_.forward_left:
            //         twist_msg_.linear.x = linear_step_;
            //         twist_msg_.linear.y = linear_step_;
            //         twist_msg_.angular.z = 0.0;
            //         break;
            //     case key_bindings_.forward_right:
            //         twist_msg_.linear.x = linear_step_;
            //         twist_msg_.linear.y = -linear_step_;
            //         twist_msg_.angular.z = 0.0;
            //         break;
            //     case key_bindings_.backward_left:
            //         twist_msg_.linear.x = -linear_step_;
            //         twist_msg_.linear.y = linear_step_;
            //         twist_msg_.angular.z = 0.0;
            //         break; 
            //     case key_bindings_.backward_right:
            //         twist_msg_.linear.x = -linear_step_;
            //         twist_msg_.linear.y = -linear_step_;
            //         twist_msg_.angular.z = 0.0;
            //         break;
            //     // 调整步长
            //     case key_bindings_.increase_linear:
            //         linear_step_ += 0.1;
            //         printf("\rLinear step increased to: %.2f", linear_step_);
            //         break;
            //     case key_bindings_.decrease_linear:
            //         linear_step_ -= 0.1;
            //         if (linear_step_ < 0.1) linear_step_ = 0.1;
            //         printf("\rLinear step decreased to: %.2f", linear_step_);
            //         break;
            //     case key_bindings_.increase_angular:
            //         angular_step_ += 0.01;
            //         printf("\rAngular step increased to: %.2f", angular_step_);
            //         break;
            //     case key_bindings_.decrease_angular:
            //         angular_step_ -= 0.01;
            //         if (angular_step_ < 0.01) angular_step_ = 0.01;
            //         printf("\rAngular step decreased to: %.2f", angular_step_);
            //         break;   
            //     case 3: // CTRL+C
            //         quit = true;
            //         break;  
            //     default:
            //         printf("\rUnknown command: %c", key);
            //         break;
            //}
            publishTwist(twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.angular.z);
            fflush(stdout);
        }
        ros::spinOnce();
    } 
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "teleop_twist_keyboard");
    TeleopTwistKeyboard teleop_twist_keyboard;
    teleop_twist_keyboard.run();
    return 0;
}
