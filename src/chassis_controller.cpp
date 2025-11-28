#include "../include/chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace chassis_controller {

bool ChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, 
                             ros::NodeHandle &root_nh, 
                             ros::NodeHandle &controller_nh) {
    // 初始化关节 轮子
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
    i_max_ = controller_nh.param("pivot_i_max", 10.0);
    i_max_ = controller_nh.param("pivot_i_max", 10.0);
    i_max_w_ = controller_nh.param("wheel_i_max", 10.0);
    i_min_w_ = controller_nh.param("wheel_i_min", 10.0);
    speed_ = controller_nh.param("speed", 10);

    // 设置默认PID增益
    gains_.p_gain_ = controller_nh.param("pivot_p_gain", 1.0);
    gains_.i_gain_ = controller_nh.param("pivot_i_gain", 0.0);
    gains_.d_gain_ = controller_nh.param("pivot_d_gain", 0.0);
    
    gains_wheel_.p_gain_ = controller_nh.param("wheel_p_gain", 2.0);
    gains_wheel_.i_gain_ = controller_nh.param("wheel_i_gain", 0.1);
    gains_wheel_.d_gain_ = controller_nh.param("wheel_d_gain", 0.0);

    // 初始化PID控制器
    pid_lf_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    pid_rf_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    pid_lb_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);
    pid_rb_.initPid(gains_.p_gain_, gains_.i_gain_, gains_.d_gain_, i_min_, i_max_);

    pid_lf_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_w_, i_max_w_);
    pid_rf_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_w_, i_max_w_);
    pid_lb_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_w_, i_max_w_);
    pid_rb_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, i_min_w_, i_max_w_);

    // 初始化速度命令
    current_cmd_.linear.x = 0.0;
    current_cmd_.linear.y = 0.0;
    current_cmd_.angular.z = 0.0;
    received_cmd_ = false;

    // double speed = 8;
    // // forward
    // double theta_fwd[4] = {0.0, 0.0, 0.0, 0.0};
    // double v_fwd[4] = {speed, speed, speed, speed};

    // // backward
    // double theta_bwd[4] = {0.0, 0.0, 0.0, 0.0};
    // double v_bwd[4] = {-speed, -speed, -speed, -speed};

    // //  left strafe
    // double theta_left[4] = {M_PI/2, M_PI/2, M_PI/2, M_PI/2};
    // double v_left[4] = {speed, speed, speed, speed};

    // // right strafe
    // double theta_right[4] = {-M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2};
    // double v_right[4] = {speed, speed, speed, speed};
    
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
    // 万向轮运动学模型
    // 每个轮子的速度是机器人速度在该轮子方向上的投影
    
    // 假设四个万向轮呈X形布置，角度分别为45°, 135°, 225°, 315°
    const double angle_front_left = M_PI/4.0;    // 45°
    const double angle_front_right = 3*M_PI/4.0; // 135°
    const double angle_back_left = -3*M_PI/4.0;  // -135° 或 225°
    const double angle_back_right = -M_PI/4.0;   // -45° 或 315°
    
    // 计算每个轮子的线速度
    // 轮子速度 = vx * cos(θ) + vy * sin(θ) + omega * R
    // 其中R是轮子到机器人中心的距离
    double R = sqrt(wheel_track_*wheel_track_ + wheel_base_*wheel_base_) / 2.0;
    
    wheel_cmd_[0] = vx * cos(angle_front_left) + vy * sin(angle_front_left) + omega * R;  // 左前
    wheel_cmd_[1] = vx * cos(angle_front_right) + vy * sin(angle_front_right) + omega * R; // 右前
    wheel_cmd_[2] = vx * cos(angle_back_left) + vy * sin(angle_back_left) + omega * R;    // 左后
    wheel_cmd_[3] = vx * cos(angle_back_right) + vy * sin(angle_back_right) + omega * R;  // 右后
    
    // 万向轮没有转向，所以转向命令应该为0
    for (int i = 0; i < 4; i++) {
        pivot_cmd_[i] = 0.0;
    }
    
    // 限制最大速度
    double max_speed = speed_;
    for (int i = 0; i < 4; ++i) {
        if (wheel_cmd_[i] > max_speed) wheel_cmd_[i] = max_speed;
        if (wheel_cmd_[i] < -max_speed) wheel_cmd_[i] = -max_speed;
    }
}

void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
    // 检查是否收到过速度命令
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
    
    // 只控制轮子速度，不控制转向
    // 应用PID控制到轮子速度
    front_left_wheel_joint_.setCommand(pid_lf_wheel_.computeCommand(wheel_cmd_[0] - front_left_wheel_joint_.getVelocity(), period));
    front_right_wheel_joint_.setCommand(pid_rf_wheel_.computeCommand(wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), period));
    back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand(wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), period));
    back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand(wheel_cmd_[3] - back_right_wheel_joint_.getVelocity(), period));

    // 万向轮不需要转向控制，注释掉以下转向控制代码
    /*
    front_left_pivot_joint_.setCommand(pid_lf_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period));
    front_right_pivot_joint_.setCommand(pid_rf_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period));
    back_left_pivot_joint_.setCommand(pid_lb_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period));
    back_right_pivot_joint_.setCommand(pid_rb_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period));
    */
}

// 插件导出宏必须放在命名空间外部
PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)

} // namespace chassis_controller