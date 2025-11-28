#include "../include/chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace chassis_controller {

bool ChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, 
                            ros::NodeHandle &root_nh, 
                            ros::NodeHandle &controller_nh) {
    // 初始化关节
    try {
        front_left_pivot_joint_ = effort_joint_interface->getHandle("left_front_pivot_joint");
        front_right_pivot_joint_ = effort_joint_interface->getHandle("right_front_pivot_joint");
        back_left_pivot_joint_ = effort_joint_interface->getHandle("left_back_pivot_joint");
        back_right_pivot_joint_ = effort_joint_interface->getHandle("right_back_pivot_joint");

        front_left_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
    }

    // 初始化轮距和轴距
    wheel_track_ = controller_nh.param("wheel_track", 0.362);
    wheel_base_ = controller_nh.param("wheel_base", 0.362);

    // 初始化PID参数
    i_max_ = 10.0;  // 设置合理的默认值
    i_min_ = -10.0;
    speed_ = 1.0;

    // 设置默认PID增益
    gains_.p_gain_ = 1.0;
    gains_.i_gain_ = 0.0;
    gains_.d_gain_ = 0.0;
    
    gains_wheel_.p_gain_ = 2.0;
    gains_wheel_.i_gain_ = 0.1;
    gains_wheel_.d_gain_ = 0.0;

    // 初始化PID控制器
    pid_lf_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    pid_rf_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    pid_lb_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    pid_rb_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);

    pid_lf_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
    pid_rf_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
    pid_lb_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
    pid_rb_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);

    // 初始化速度命令
    current_cmd_.linear.x = 0.0;
    current_cmd_.linear.y = 0.0;
    current_cmd_.angular.z = 0.0;
    received_cmd_ = false;
    
    // 创建订阅器
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisController::cmdVelCallback, this);

    ROS_INFO("ChassisController initialized successfully");
    return true;  // 必须返回true
}

void ChassisController::setSpeed(double speed) {
    speed_ = speed;
}

void ChassisController::updateGains(double kp, double ki, double kd, double i_max, double i_min, bool is_wheel) {
    i_max_ = i_max;
    i_min_ = i_min;
    
    if (is_wheel) {
        gains_wheel_.p_gain_ = kp;
        gains_wheel_.i_gain_ = ki;
        gains_wheel_.d_gain_ = kd;
        
        // 重新初始化轮子PID
        pid_lf_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
        pid_rf_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
        pid_lb_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
        pid_rb_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_, i_max_);
    } else {
        gains_.p_gain_ = kp;
        gains_.i_gain_ = ki;
        gains_.d_gain_ = kd;
        
        // 重新初始化转向PID
        pid_lf_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
        pid_rf_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
        pid_lb_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
        pid_rb_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    }
}

void ChassisController::starting(const ros::Time &time) {
    last_cmd_time_ = time;
    ROS_INFO("ChassisController started");
}

void ChassisController::stopping(const ros::Time &time) {
    for (int i = 0; i < 4; i++) {
        pivot_cmd_[i] = 0;
        wheel_cmd_[i] = 0;
    }
    ROS_INFO("ChassisController stopped");
}

void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    current_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    received_cmd_ = true;
}

void ChassisController::calculateWheelCommands(double vx, double vy, double omega) {
    // 全向移动底盘运动学模型
    // 计算各轮子的线速度分量
    double v_front_left = vx - vy - omega * (wheel_track_ + wheel_base_) / 2.0;
    double v_front_right = vx + vy + omega * (wheel_track_ + wheel_base_) / 2.0;
    double v_back_left = vx + vy - omega * (wheel_track_ + wheel_base_) / 2.0;
    double v_back_right = vx - vy + omega * (wheel_track_ + wheel_base_) / 2.0;
    
    // 对于麦克纳姆轮，转向角度固定为45°
    const double fixed_angle = M_PI / 4.0;
    
    pivot_cmd_[0] = fixed_angle;   // 左前轮
    pivot_cmd_[1] = -fixed_angle;  // 右前轮  
    pivot_cmd_[2] = -fixed_angle;  // 左后轮
    pivot_cmd_[3] = fixed_angle;   // 右后轮
    
    // 设置轮子速度
    wheel_cmd_[0] = v_front_left;
    wheel_cmd_[1] = v_front_right;
    wheel_cmd_[2] = v_back_left;
    wheel_cmd_[3] = v_back_right;
    
    // 限制最大速度
    double max_speed = 10.0;
    for (int i = 0; i < 4; ++i) {
        if (wheel_cmd_[i] > max_speed) wheel_cmd_[i] = max_speed;
        if (wheel_cmd_[i] < -max_speed) wheel_cmd_[i] = -max_speed;
    }
}

void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
    // 检查是否收到过速度命令，如果没有或者超时（如0.5秒），则停止
    bool active_cmd = false;
    geometry_msgs::Twist current_cmd;
    
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (received_cmd_ && (time - last_cmd_time_).toSec() < 0.5) {
            current_cmd = current_cmd_;
            active_cmd = true;
        }
    }
    
    if (!active_cmd) {
        // 没有有效命令，停止机器人
        current_cmd.linear.x = 0.0;
        current_cmd.linear.y = 0.0;
        current_cmd.angular.z = 0.0;
    }
    
    // 根据速度命令计算轮子命令
    calculateWheelCommands(current_cmd.linear.x, current_cmd.linear.y, current_cmd.angular.z);
    
    // 应用PID控制到轮子速度
    front_left_wheel_joint_.setCommand(pid_lf_wheel_.computeCommand(wheel_cmd_[0] - front_left_wheel_joint_.getVelocity(), period));
    front_right_wheel_joint_.setCommand(pid_rf_wheel_.computeCommand(wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), period));
    back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand(wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), period));
    back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand(wheel_cmd_[3] - back_right_wheel_joint_.getVelocity(), period));

    // 应用PID控制到转向角度
    front_left_pivot_joint_.setCommand(pid_lf_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period));
    front_right_pivot_joint_.setCommand(pid_rf_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period));
    back_left_pivot_joint_.setCommand(pid_lb_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period));
    back_right_pivot_joint_.setCommand(pid_rb_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period));
}

// 插件导出宏必须放在命名空间外部
PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)

} // namespace chassis_controller