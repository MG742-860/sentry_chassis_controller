#include "../include/sentry_chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace sentry_chassis_controller {
    bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
        
        //初始化关节
        front_left_pivot_joint_ = effort_joint_interface->getHandle("left_front_pivot_joint");
        front_right_pivot_joint_ = effort_joint_interface->getHandle("right_front_pivot_joint");
        back_left_pivot_joint_ = effort_joint_interface->getHandle("left_back_pivot_joint");
        back_right_pivot_joint_ = effort_joint_interface->getHandle("right_back_pivot_joint");

        front_left_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        //设置PID参数
        gains_pivot_.p_gain_ = controller_nh.param("pivot_p_gain", 1.0);
        gains_pivot_.i_gain_ = controller_nh.param("pivot_i_gain", 0.0);
        gains_pivot_.d_gain_ = controller_nh.param("pivot_d_gain", 0.0);
        gains_pivot_.i_max_ = controller_nh.param("pivot_i_max", 0.0);
        gains_pivot_.i_min_ = controller_nh.param("pivot_i_min", 0.0);

        gains_wheel_.p_gain_ = controller_nh.param("wheel_p_gain", 2.0);
        gains_wheel_.i_gain_ = controller_nh.param("wheel_i_gain", 0.1);
        gains_wheel_.d_gain_ = controller_nh.param("wheel_d_gain", 0.0);
        gains_wheel_.i_max_ = controller_nh.param("wheel_i_max", 0.0);
        gains_wheel_.i_min_ = controller_nh.param("wheel_i_min", 0.0);

        //初始化PID控制器
        pid_fl_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_fr_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_bl_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_br_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);

        pid_fl_wheel_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_fr_wheel_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_bl_wheel_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_br_wheel_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);

        //底盘和最大速度
        //底盘：是否锁住，也就是朝向是否改变，false为改变，true为不改变
        is_forward_lock_ = controller_nh.param("is_forward_lock", false);
        max_speed_ = controller_nh.param("speed", 10);
        max_angular_ = controller_nh.param("angular", 10);

        //初始化停止时间阈值
        stop_time_ = controller_nh.param("stop_time", 0.5);

        //初始化 twist消息
        gotten_msg.linear.x = 0;
        gotten_msg.linear.y = 0;
        gotten_msg.angular.z = 0;
        received_msg_ = false;
        //初始化订阅者
        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SentryChassisController::CmdVelCallback, this);
        ROS_INFO("Sentry Chassis Controller initialized");

        return true;
    }

    void SentryChassisController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg){
        gotten_msg = *msg;
        received_msg_ = true;
        last_time_ = ros::Time::now();
    }

    void SentryChassisController::calculateWheelCommands(double vx, double vy, double angular, bool is_lock){
        //先判断键盘是否有消息
        if (!received_msg_)
        {
            //如果没有，就停止小车
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = 0;
                pivot_cmd_[i] = 0;
            }
            return;
        }
        //如果有消息，判断是否超过设定的最大值
        if (abs(vx) > abs(max_speed_))
        {
            vx = max_speed_ * (abs(vx) / vx);
        }
        if (abs(angular) > abs(max_angular_))
        {
            angular = max_angular_ * (abs(angular) / angular);
        }
                

        if (is_forward_lock_)
        {
            //朝向锁住，先将angular速度归零
            angular = 0;

            //开始计算角度
            angular = atan2(vy, vx);
            double speed = sqrt(vx * vx + vy * vy);

            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = speed;
                pivot_cmd_[i] = angular;
            }
            
            return;
        }
        else
        {
            //底盘没有锁住，y方向速度归零
            vy = 0;
        }
        
        //底盘没有锁住，不需要转动关节
        for (int i = 0; i < 4; i++)
        {
            pivot_cmd_[i] = 0;
        }
        //设置x方向速度
        if (vx)
        {
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = vx;
            }
        }
        //然后加权后设置转向速度
        if (angular)
        {
            wheel_cmd_[0] -= angular;
            wheel_cmd_[2] -= angular;
            wheel_cmd_[1] += angular;
            wheel_cmd_[3] += angular;
        }
    }

    void SentryChassisController::update(const ros::Time &time, const ros::Duration &period){

        //如果时间阈值内键盘没有输入正确的指令，就停止小车
        if ((time - last_time_).toSec() > stop_time_)
        {
            received_msg_ = false;
            last_time_ = time;
        }

        calculateWheelCommands(gotten_msg.linear.x, gotten_msg.linear.y,gotten_msg.angular.z, is_forward_lock_);

        front_left_wheel_joint_.setCommand(pid_fl_wheel_.computeCommand(wheel_cmd_[0]/*6*/ - front_left_wheel_joint_.getVelocity(), period));
        front_right_wheel_joint_.setCommand(pid_fr_wheel_.computeCommand(wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), period));
        back_left_wheel_joint_.setCommand(pid_bl_wheel_.computeCommand(wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), period));
        back_right_wheel_joint_.setCommand(pid_br_wheel_.computeCommand(wheel_cmd_[3] - back_right_wheel_joint_.getVelocity(), period));

        front_left_pivot_joint_.setCommand(pid_fl_pivot_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period));
        front_right_pivot_joint_.setCommand(pid_fr_pivot_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period));
        back_left_pivot_joint_.setCommand(pid_bl_pivot_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period));
        back_right_pivot_joint_.setCommand(pid_br_pivot_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period));

    }
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
