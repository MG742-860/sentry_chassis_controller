#include "../include/sentry_chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

// CPP文件函数定义前后顺序与头文件一致

namespace sentry_chassis_controller {

    void SentryChassisController::get_joint(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        front_left_pivot_joint_ = effort_joint_interface->getHandle("left_front_pivot_joint");
        front_right_pivot_joint_ = effort_joint_interface->getHandle("right_front_pivot_joint");
        back_left_pivot_joint_ = effort_joint_interface->getHandle("left_back_pivot_joint");
        back_right_pivot_joint_ = effort_joint_interface->getHandle("right_back_pivot_joint");

        front_left_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
    }

    void SentryChassisController::get_parameters(ros::NodeHandle &controller_nh)
    {
        //轮距轴距
        wheel_track_ = controller_nh.param("wheel_track", 0.5);
        wheel_base_ = controller_nh.param("wheel_base", 0.5);
        wheel_radius_ = controller_nh.param("wheel_radius", 0.0762);
        wheel_state_[0].x_ = -wheel_base_/2, wheel_state_[0].y_ = wheel_track_/2;
        wheel_state_[1].x_ = wheel_base_/2, wheel_state_[1].y_ = wheel_track_/2;
        wheel_state_[2].x_ = -wheel_base_/2, wheel_state_[2].y_ = -wheel_track_/2;
        wheel_state_[3].x_ = wheel_base_/2, wheel_state_[3].y_ = -wheel_track_/2;
        for (int i = 0; i < 4; i++)
        {
            wheel_state_[i].direction_m_ = atan2(wheel_state_[i].y_,wheel_state_[i].x_);
        }
        
        //速度、角度、转角最大值、加速度限制
        max_speed_ = controller_nh.param("max_speed", 1.0);
        max_angular_ = controller_nh.param("max_angular", 1.0);
        double max_direction_c = controller_nh.param("max_direction", 90.0);
        max_direction_ = max_direction_c * M_PI / 180.0; // 转化为弧度
        motion_limits_.is_enable = controller_nh.param("enable_acceleration_limits", false);
        motion_limits_.max_linear_acceleration = controller_nh.param("max_linear_acceleration", 3.0);
        motion_limits_.max_angular_acceleration = controller_nh.param("max_angular_acceleration", 5.0);
        motion_limits_.max_linear_deceleration = controller_nh.param("max_linear_deceleration", 5.0);
        motion_limits_.max_angular_deceleration = controller_nh.param("max_angular_deceleration", 8.0);
        //小陀螺系数
        speed_to_rotate_ = controller_nh.param("speed_to_rotate", 100.0);
        //履带模式差速系数
        speed_diff_m_ = controller_nh.param("speed_diff_m",10);
        //没有指令时停止小车阈值
        stop_time_ = controller_nh.param("stop_time", 0.5);
        //是否打印相关信息
        print_expected_speed_ = controller_nh.param("print_expected_speed", false);
        print_expected_pivot_ = controller_nh.param("print_expected_pivot", false);
        odom_show_ = controller_nh.param("print_odom", false);
        //里程计相关参数        
        tf_publish_period_ = controller_nh.param("tf_publish_period", 0.05);
        publish_odom_ = controller_nh.param("publish_odom", true);
        publish_tf_ = controller_nh.param("publish_tf", true);
        odom_frame_id_ = controller_nh.param("odom_frame_id", std::string("odom"));
        base_frame_id_ = controller_nh.param("base_frame_id", std::string("base_link"));
        //坐标系选择
        coordinate_mode_ = controller_nh.param("coordinate_mode", false); // false: 底盘坐标系，true: 全局坐标系

        // 驱动模式和转向模式
        int drive_mode = controller_nh.param("drive_mode", 2);  // 默认全驱
        int turn_mode = controller_nh.param("turn_mode", 2);    // 默认全轮转向
        //转化为枚举类型
        switch (drive_mode) 
        {
            case 0:
                drive_mode_ = DriveMode::ForwardDrive;
                break;
            case 1:
                drive_mode_ = DriveMode::BackwardDrive;
                break;
            case 2:
                drive_mode_ = DriveMode::AllDrive;
                break;
            default:
                drive_mode_ = DriveMode::AllDrive;
                break;
        }
        switch (turn_mode) 
        {
            case 0:
                turn_mode_ = TurnMode::ForwardTurn;
                break;
            case 1:
                turn_mode_ = TurnMode::BackwardTurn;
                break;
            case 2:
                turn_mode_ = TurnMode::AllTurn;
                break;
            case 3:
                turn_mode_ = TurnMode::NoneTurn;
                break;
            default:
                turn_mode_ = TurnMode::AllTurn;
                break;
        }
        //全向一定是全驱
        if (turn_mode_ == TurnMode::AllTurn) {
            drive_mode_ = DriveMode::AllDrive;
        }
        //初始化当前状态
        current_speed_ = 0.0;
        current_direction_ = 0.0;
        current_angular_ = 0.0;
    }

    void SentryChassisController::get_pid_parameters(ros::NodeHandle &controller_nh)
    {
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
    }

    void SentryChassisController::init_pid_parameters(ros::NodeHandle &controller_nh)
    {
        pid_fl_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_fr_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_bl_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);
        pid_br_pivot_.initPid(gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_, gains_pivot_.i_min_, gains_pivot_.i_max_);

        pid_fl_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, gains_wheel_.i_min_, gains_wheel_.i_max_);
        pid_fr_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, gains_wheel_.i_min_, gains_wheel_.i_max_);
        pid_bl_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, gains_wheel_.i_min_, gains_wheel_.i_max_);
        pid_br_wheel_.initPid(gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_, gains_wheel_.i_min_, gains_wheel_.i_max_);
    }

    void SentryChassisController::setSpeedPivot(const ros::Duration &period)
    {
        front_left_pivot_joint_.setCommand(pid_fl_pivot_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period));
        front_right_pivot_joint_.setCommand(pid_fr_pivot_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period));
        back_left_pivot_joint_.setCommand(pid_bl_pivot_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period));
        back_right_pivot_joint_.setCommand(pid_br_pivot_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period));

        front_left_wheel_joint_.setCommand(pid_fl_wheel_.computeCommand(wheel_cmd_[0]/*6*/ - front_left_wheel_joint_.getVelocity(), period));
        front_right_wheel_joint_.setCommand(pid_fr_wheel_.computeCommand(wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), period));
        back_left_wheel_joint_.setCommand(pid_bl_wheel_.computeCommand(wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), period));
        back_right_wheel_joint_.setCommand(pid_br_wheel_.computeCommand(wheel_cmd_[3] - back_right_wheel_joint_.getVelocity(), period));
    }

    void SentryChassisController::reset_pid()
    {
        pid_fl_pivot_.reset();
        pid_fr_pivot_.reset();
        pid_bl_pivot_.reset();
        pid_br_pivot_.reset();

        pid_fl_wheel_.reset();
        pid_fr_wheel_.reset();
        pid_bl_wheel_.reset();
        pid_br_wheel_.reset();
    }

    void SentryChassisController::dynamicReconfigureCallback(sentry_chassis_controller::SentryChassisConfig &config, uint32_t level)
    {
        setlocale(LC_ALL, "");
        ROS_INFO("动态参数更新 (level: %d)", level);
        
        // 更新轮子参数
        wheel_track_ = config.wheel_track;
        wheel_base_ = config.wheel_base;
        wheel_radius_ = config.wheel_radius;
        
        // 更新转向关节PID参数
        gains_pivot_.p_gain_ = config.pivot_p_gain;
        gains_pivot_.i_gain_ = config.pivot_i_gain;
        gains_pivot_.d_gain_ = config.pivot_d_gain;
        gains_pivot_.i_max_ = config.pivot_i_max;
        gains_pivot_.i_min_ = config.pivot_i_min;
        
        // 更新驱动轮PID参数
        gains_wheel_.p_gain_ = config.wheel_p_gain;
        gains_wheel_.i_gain_ = config.wheel_i_gain;
        gains_wheel_.d_gain_ = config.wheel_d_gain;
        gains_wheel_.i_max_ = config.wheel_i_max;
        gains_wheel_.i_min_ = config.wheel_i_min;
        
        // 更新运动限制参数
        max_speed_ = config.max_speed;
        max_angular_ = config.max_angular;
        max_direction_ = config.max_direction * M_PI / 180.0;  // 度转弧度
        motion_limits_.is_enable = config.enable_acceleration_limits;
        motion_limits_.max_angular_acceleration = config.max_angular_acceleration;
        motion_limits_.max_linear_acceleration = config.max_linear_acceleration;
        motion_limits_.max_angular_deceleration = config.max_angular_deceleration;
        motion_limits_.max_linear_deceleration = config.max_linear_deceleration;
        stop_time_ = config.stop_time;
        
        // 更新小陀螺系数控制和履带模式差速系数
        speed_to_rotate_ = config.speed_to_rotate;
        speed_diff_m_ = config.speed_diff_m;
        
        // 更新功率参数
        power_mgt_.max_power_ = config.max_power;
        power_mgt_.is_enable_ = config.enable_power_limit;
        power_mgt_.is_print_ = config.enable_power_print;
        power_mgt_.k1_ = config.k1;
        power_mgt_.k2_ = config.k2;
        power_mgt_.torque_constant_ = config.torque_constant;
        
        // 更新坐标系模式
        coordinate_mode_ = config.coordinate_mode;
        
        // 更新驱动和转向模式
        drive_mode_ = static_cast<DriveMode>(config.drive_mode);
        turn_mode_ = static_cast<TurnMode>(config.turn_mode);
        
        // 更新调试选项
        print_expected_speed_ = config.print_expected_speed;
        print_expected_pivot_ = config.print_expected_pivot;
        odom_show_ = config.print_odom;
        
        // 重新初始化PID控制器
        init_pid_parameters(nh_);
        
        ROS_INFO("PID参数已更新:");
        ROS_INFO("  转向关节 - P:%.2f, I:%.2f, D:%.2f", 
                gains_pivot_.p_gain_, gains_pivot_.i_gain_, gains_pivot_.d_gain_);
        ROS_INFO("  驱动轮 - P:%.2f, I:%.2f, D:%.2f", 
                gains_wheel_.p_gain_, gains_wheel_.i_gain_, gains_wheel_.d_gain_);
        ROS_INFO("  驱动模式: %d, 转向模式: %d", config.drive_mode, config.turn_mode);
        ROS_INFO("  坐标系模式: %s", coordinate_mode_ ? "全局坐标系" : "底盘坐标系");
    }

    void SentryChassisController::publishJointStates(){
        // 更新关节状态
        ros::Time time = ros::Time::now();
        joint_state_msg_.header.stamp = time;

        // 获取关节位置和速度
        joint_state_msg_.position[0] = front_left_pivot_joint_.getPosition();
        joint_state_msg_.position[1] = front_right_pivot_joint_.getPosition();
        joint_state_msg_.position[2] = back_left_pivot_joint_.getPosition();
        joint_state_msg_.position[3] = back_right_pivot_joint_.getPosition();

        joint_state_msg_.position[4] = front_left_wheel_joint_.getPosition();
        joint_state_msg_.position[5] = front_right_wheel_joint_.getPosition();
        joint_state_msg_.position[6] = back_left_wheel_joint_.getPosition();
        joint_state_msg_.position[7] = back_right_wheel_joint_.getPosition();

        joint_state_msg_.velocity[0] = front_left_pivot_joint_.getVelocity();
        joint_state_msg_.velocity[1] = front_right_pivot_joint_.getVelocity();
        joint_state_msg_.velocity[2] = back_left_pivot_joint_.getVelocity();
        joint_state_msg_.velocity[3] = back_right_pivot_joint_.getVelocity();

        joint_state_msg_.velocity[4] = front_left_wheel_joint_.getVelocity();
        joint_state_msg_.velocity[5] = front_right_wheel_joint_.getVelocity();
        joint_state_msg_.velocity[6] = back_left_wheel_joint_.getVelocity();
        joint_state_msg_.velocity[7] = back_right_wheel_joint_.getVelocity();

        // 发布关节状态
        joint_state_pub_.publish(joint_state_msg_);
    }

    void SentryChassisController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        gotten_msg = *msg;
        last_angular_ = (gotten_msg.angular.z && (fabs(gotten_msg.angular.z) - fabs(M_PI/4)) > 1e-2)? gotten_msg.angular.z : last_angular_;
        received_msg_ = true;
        last_update_time_ = ros::Time::now();
    }

    void SentryChassisController::convertToRobotFrame(double& robot_x, double& robot_y, double& robot_angular)
    {
        if (coordinate_mode_)// 全局坐标系才进行转换
        {
            // 全局坐标系，进行转换
            double theta = rob_state_.theta;// 获取当前机器人朝向角
            // 先获取原始值
            double global_x = robot_x;
            double global_y = robot_y;
            // 进行坐标变换
            robot_x = global_x * cos(theta) + global_y * sin(theta);
            robot_y = -global_x * sin(theta) + global_y * cos(theta);
            // 角速度在全局和底盘坐标系下相同，无需转换
            return;
        }
    }

    void SentryChassisController::calculateWheel(const double direction_src, const double speed_src,wheel_state wheel_state_dd[])
    {
        double speed = speed_src / wheel_radius_;//目标速度转化为轮子转速
        // 根据驱动模式调整轮子速度
        switch (drive_mode_)
        {
            case ForwardDrive://前驱
                wheel_cmd_[0] += (fabs(wheel_state_dd[0].speed_)/wheel_radius_)*(fabs(direction_src)>M_PI_2?(-1):1);//左前
                wheel_cmd_[1] += (fabs(wheel_state_dd[1].speed_)/wheel_radius_)*(fabs(direction_src)>M_PI_2?(-1):1);//右前
                wheel_cmd_[2] = 0;//左后
                wheel_cmd_[3] = 0;//右后
                break;
            case BackwardDrive://后驱
                wheel_cmd_[0] = 0;//左前
                wheel_cmd_[1] = 0;//右前
                wheel_cmd_[2] += (fabs(wheel_state_dd[2].speed_)/wheel_radius_)*(fabs(direction_src)>M_PI_2?(-1):1);//左后
                wheel_cmd_[3] += (fabs(wheel_state_dd[3].speed_)/wheel_radius_)*(fabs(direction_src)>M_PI_2?(-1):1);//右后
                break;
            case AllDrive://全驱(默认)
            default:
                for (int i = 0; i < 4; i++)
                {
                    wheel_cmd_[i] += (turn_mode_ == AllTurn)?fabs(wheel_state_dd[i].speed_/wheel_radius_):(fabs(wheel_state_dd[i].speed_)/wheel_radius_)*(fabs(direction_src)>M_PI_2?(-1):1);
                }
                break;
        }
    }

    void SentryChassisController::calculatePivot(const double direction_src, const double angular, wheel_state wheel_state_dd[])//传入的是方向角、角速度
    {
        double direction_limit[4] = {direction_src};
        // 根据转向模式调整轮子转向角度
        switch (turn_mode_)
        {
            case ForwardTurn://前轮转向
                for (int i = 0; i < 2; i++)
                {
                    direction_limit[i] = wheel_state_dd[i].direction_;
                    if(fabs(direction_limit[i])>M_PI_2)
                    {
                        direction_limit[i] = M_PI*(direction_limit[i]/fabs(direction_limit[i])) - direction_limit[i];
                    }
                    direction_limit[i] = fabs(direction_limit[i])<max_direction_?direction_limit[i]:max_direction_*
                                                        (direction_limit[i]/fabs(direction_limit[i]));
                } 
                pivot_cmd_[0] = direction_limit[0];//左前
                pivot_cmd_[1] = direction_limit[1];//右前
                pivot_cmd_[2] = 0;//左后
                pivot_cmd_[3] = 0;//右后
                break;
            case BackwardTurn://后轮转向
                for (int i = 2; i < 4; i++)
                {
                    direction_limit[i] = wheel_state_dd[i].direction_;
                    if(fabs(direction_limit[i])>M_PI_2)
                    {
                        direction_limit[i] = M_PI*(direction_limit[i]/fabs(direction_limit[i])) - direction_limit[i];
                    }
                    direction_limit[i] = fabs(direction_limit[i])<max_direction_?direction_limit[i]:max_direction_*(
                                                        direction_limit[i]/fabs(direction_limit[i]));
                } 
                pivot_cmd_[0] = 0;//左前
                pivot_cmd_[1] = 0;//右前
                pivot_cmd_[2] = -direction_limit[2];//左后
                pivot_cmd_[3] = -direction_limit[3];//右后
                break;
            case AllTurn://全轮转向(默认)
                for (int i = 0; i < 4; i++)
                {
                    pivot_cmd_[i] = wheel_state_dd[i].direction_;
                }         
                break;
            case NoneTurn://不转向
                for (int i = 0; i < 4; i++)
                {
                    pivot_cmd_[i] = 0;
                }
                //然后应用差速转向(修改轮子速度)
                handleDifferentialSteering(direction_src);
                break;
            default:
                // 不做任何限制
                break;
        }
    }

    void SentryChassisController::handleDifferentialSteering(const double angular)
    {
        if (fabs(angular) < 1e-6) 
        {
            return;  // 没有角速度，不需要差速
        }

        double speed_diff = angular * (wheel_track_ / 2.0) * speed_diff_m_;
        
        // 根据驱动模式应用差速
        switch (drive_mode_) 
        {
            case ForwardDrive:  // 前驱差速
                wheel_cmd_[0] -= speed_diff/wheel_radius_;  // 左前轮减速
                wheel_cmd_[1] += speed_diff/wheel_radius_;  // 右前轮加速
                break;
                
            case BackwardDrive:  // 后驱差速
                wheel_cmd_[2] -= speed_diff/wheel_radius_;  // 左后轮减速
                wheel_cmd_[3] += speed_diff/wheel_radius_;  // 右后轮加速
                break;
                
            case AllDrive:  // 四驱差速
            default:
                wheel_cmd_[0] -= speed_diff/wheel_radius_;  // 左前轮减速
                wheel_cmd_[1] += speed_diff/wheel_radius_;  // 右前轮加速
                wheel_cmd_[2] -= speed_diff/wheel_radius_;  // 左后轮减速
                wheel_cmd_[3] += speed_diff/wheel_radius_;  // 右后轮加速
                break;
        }
    }

    void SentryChassisController::applyAccelerationLimits(double& vx_target, double& vy_target, double& omega_target, const ros::Duration& period)
    {
        // 计算速度变化量
        double delta_vx = vx_target - motion_limits_.last_vx_;
        double delta_vy = vy_target - motion_limits_.last_vy_;
        double delta_omega = omega_target - motion_limits_.last_omega_;
        
        // 计算合速度大小
        double current_speed = sqrt(motion_limits_.last_vx_*motion_limits_.last_vx_ + motion_limits_.last_vy_*motion_limits_.last_vy_);
        double target_speed = sqrt(vx_target*vx_target + vy_target*vy_target);
        
        // 判断是加速还是减速
        bool is_accelerating = (target_speed > current_speed);
        double max_linear_accel = is_accelerating ? 
            motion_limits_.max_linear_acceleration : motion_limits_.max_linear_deceleration;
        double max_angular_accel = is_accelerating ? 
            motion_limits_.max_angular_acceleration : motion_limits_.max_angular_deceleration;
        
        // 限制线加速度
        double max_delta_linear = max_linear_accel * period.toSec();
        double delta_speed = sqrt(delta_vx*delta_vx + delta_vy*delta_vy);
        
        if (delta_speed > max_delta_linear) {
            // 按比例缩放速度增量
            double scale = max_delta_linear / delta_speed;
            vx_target = motion_limits_.last_vx_ + delta_vx * scale;
            vy_target = motion_limits_.last_vy_ + delta_vy * scale;
        }
        
        // 限制角加速度
        double max_delta_angular = max_angular_accel * period.toSec();
        if (fabs(delta_omega) > max_delta_angular) {
            omega_target = motion_limits_.last_omega_ + copysign(max_delta_angular, delta_omega);
        }
        
        // 更新记录的速度值
        motion_limits_.last_vx_ = vx_target;
        motion_limits_.last_vy_ = vy_target;
        motion_limits_.last_omega_ = omega_target;
    }

    void SentryChassisController::calculateWheelCommands(double vx, double vy, double angular)
    {
        static wheel_state wheel_state_d[4];
        for (int i = 0; i < 4; i++)
        {
            wheel_state_d[i] = wheel_state_[i];
        }
        
        //先判断键盘是否有消息
        if (!received_msg_)
        {
            //如果没有，就自锁小车
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = 0;//速度归零
                pivot_cmd_[i] = wheel_state_d[i].direction_m_;//轮子正交
            }
            //不打印预期值，避免终端信息污染
            return;
        }
        //如果有消息，判断是否超过设定的最大值
        if (fabs(vx) > fabs(max_speed_))
        {
            vx = max_speed_ * (fabs(vx) / vx);
        }
        if (fabs(vy) > fabs(max_speed_))
        {
            vy = max_speed_ * (fabs(vy) / vy);
        }
        if (fabs(angular) > fabs(max_angular_))
        {
            angular = max_angular_ * (fabs(angular) / angular);
        }       

        //如果有角速度但是没有线速度，就原地转圈
        if ((fabs(angular) > 1e-6) && (sqrt(vx*vx + vy*vy) < 1e-6))
        {
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = (sqrt(pow(angular*(wheel_base_/2), 2)+pow(angular*(wheel_track_/2), 2))*(angular>0?1:-1)*speed_to_rotate_)/wheel_radius_;
                pivot_cmd_[i] = wheel_state_[i].direction_m_;//轮子正交
            }
            printExpectedSpeed();
            return;
        }

        //计算方向和速度
        //整体
        double direction = atan2(vy, vx);
        double speed = sqrt(vx * vx + vy * vy) * (fabs(direction) > M_PI_2 ? -1:1);
        //单个轮子
        for (int i = 0; i < 4; i++)
        {
            wheel_state_d[i].vx_ = vx - angular*wheel_state_d[i].y_;
            wheel_state_d[i].vy_ = vy - angular*wheel_state_d[i].x_;
            wheel_state_d[i].direction_ = atan2(wheel_state_d[i].vy_, wheel_state_d[i].vx_);
            wheel_state_d[i].speed_ = sqrt(pow(wheel_state_d[i].vx_,2)+pow(wheel_state_d[i].vy_,2))*(fabs(wheel_state_d[i].direction_) > M_PI_2 ? -1:1);
        }
        //更新轮子数据
        for (int i = 0; i < 4; i++)
        {
            wheel_state_[i] = wheel_state_d[i];
        }
        
        //先将数据置零
        for (int i = 0; i < 4; i++)
        {
            pivot_cmd_[i] = 0;
            wheel_cmd_[i] = 0;
        }
        calculatePivot(direction, angular, wheel_state_d);
        calculateWheel(direction, speed, wheel_state_d);
        printExpectedSpeed();
        return;
    }

    void SentryChassisController::calculateOdometry(const ros::Duration& period)
    {
        // 1. 获取实际传感器数据（编码器读数）
        double fl_wheel_omega = front_left_wheel_joint_.getVelocity();  // 左前轮角速度 rad/s
        double fr_wheel_omega = front_right_wheel_joint_.getVelocity(); // 右前轮角速度 rad/s
        double bl_wheel_omega = back_left_wheel_joint_.getVelocity();   // 左后轮角速度 rad/s
        double br_wheel_omega = back_right_wheel_joint_.getVelocity();  // 右后轮角速度 rad/s
        
        // 2. 将角速度转换为线速度 (m/s)
        double fl_wheel_vel = fl_wheel_omega * wheel_radius_;
        double fr_wheel_vel = fr_wheel_omega * wheel_radius_;
        double bl_wheel_vel = bl_wheel_omega * wheel_radius_;
        double br_wheel_vel = br_wheel_omega * wheel_radius_;
        
        // 3. 获取实际转向角度（编码器读数）
        double fl_pivot_angle = front_left_pivot_joint_.getPosition();
        double fr_pivot_angle = front_right_pivot_joint_.getPosition();
        double bl_pivot_angle = back_left_pivot_joint_.getPosition();
        double br_pivot_angle = back_right_pivot_joint_.getPosition();
        
        // 4. 使用运动学模型计算机器人速度
        // 对于每个轮子 i：v_wheel_i = [cos(θ_i), sin(θ_i)] * [vx - ω*y_i, vy + ω*x_i]
        // 我们可以构建线性方程组并求解 vx, vy, ω
        
        // 由于这是超定方程组（4个方程，3个未知数），我们使用最小二乘法
        // 构建矩阵 A (4x3) 和向量 b (4x1)
        Eigen::MatrixXd A(4, 3);
        Eigen::VectorXd b(4);
        
        // 轮子位置（相对于机器人中心）
        double x[4] = {wheel_state_[0].x_, wheel_state_[1].x_, wheel_state_[2].x_, wheel_state_[3].x_};
        double y[4] = {wheel_state_[0].y_, wheel_state_[1].y_, wheel_state_[2].y_, wheel_state_[3].y_};
        
        // 实际轮子线速度
        double v_wheel[4] = {fl_wheel_vel, fr_wheel_vel, bl_wheel_vel, br_wheel_vel};
        double theta[4] = {fl_pivot_angle, fr_pivot_angle, bl_pivot_angle, br_pivot_angle};
        
        // 填充矩阵A和向量b
        for (int i = 0; i < 4; i++) {
            // 方程：cos(θ_i)*(vx - ω*y_i) + sin(θ_i)*(vy + ω*x_i) = v_wheel_i
            A(i, 0) = cos(theta[i]);  // vx 系数
            A(i, 1) = sin(theta[i]);  // vy 系数
            A(i, 2) = -cos(theta[i]) * y[i] + sin(theta[i]) * x[i];  // ω 系数
            b(i) = v_wheel[i];
        }
        
        // 5. 使用最小二乘法求解：X = (A^T * A)^(-1) * A^T * b
        Eigen::Vector3d X;
        try {
            // 使用QR分解求解最小二乘问题（更稳定）
            X = A.householderQr().solve(b);
            
            // 检查解的有效性
            if (!X.allFinite()) {
                ROS_WARN_THROTTLE(1.0, "Invalid odometry solution, using previous values");
                // 使用上一次的速度值
                X(0) = rob_state_.vx;
                X(1) = rob_state_.vy;
                X(2) = rob_state_.omega;
            }
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(1.0, "Odometry calculation failed: %s", e.what());
            // 使用上一次的速度值
            X(0) = rob_state_.vx;
            X(1) = rob_state_.vy;
            X(2) = rob_state_.omega;
        }
        
        // 6. 更新机器人状态
        rob_state_.vx = X(0);
        rob_state_.vy = X(1);
        rob_state_.omega = X(2);
        
        // 7. 限制速度范围，避免异常值
        const double MAX_VELOCITY = max_speed_ * 1.2;  // 稍微放宽限制
        const double MAX_OMEGA = max_angular_ * 1.2;
        
        rob_state_.vx = std::max(std::min(rob_state_.vx, MAX_VELOCITY), -MAX_VELOCITY);
        rob_state_.vy = std::max(std::min(rob_state_.vy, MAX_VELOCITY), -MAX_VELOCITY);
        rob_state_.omega = std::max(std::min(rob_state_.omega, MAX_OMEGA), -MAX_OMEGA);
        
        // 8. 低通滤波，平滑速度估计
        static bool first_run = true;
        static double prev_vx = 0, prev_vy = 0, prev_omega = 0;
        const double alpha = 0.3;  // 滤波系数，值越小越平滑
        
        if (first_run) {
            prev_vx = rob_state_.vx;
            prev_vy = rob_state_.vy;
            prev_omega = rob_state_.omega;
            first_run = false;
        } else {
            rob_state_.vx = alpha * rob_state_.vx + (1 - alpha) * prev_vx;
            rob_state_.vy = alpha * rob_state_.vy + (1 - alpha) * prev_vy;
            rob_state_.omega = alpha * rob_state_.omega + (1 - alpha) * prev_omega;
            
            prev_vx = rob_state_.vx;
            prev_vy = rob_state_.vy;
            prev_omega = rob_state_.omega;
        }
        
        // 9. 积分得到位置和姿态
        double dt = period.toSec();
        if (dt > 0.0 && dt < 0.1) {  // 确保时间步长合理
            // 将机体坐标系速度转换到全局坐标系
            double cos_theta = cos(rob_state_.theta);
            double sin_theta = sin(rob_state_.theta);
            
            double delta_x = (rob_state_.vx * cos_theta - rob_state_.vy * sin_theta) * dt;
            double delta_y = (rob_state_.vx * sin_theta + rob_state_.vy * cos_theta) * dt;
            double delta_theta = rob_state_.omega * dt;
            
            rob_state_.x += delta_x;
            rob_state_.y += delta_y;
            rob_state_.theta += delta_theta;
            
            // 归一化theta到[-π, π]
            while (rob_state_.theta > M_PI) rob_state_.theta -= 2.0 * M_PI;
            while (rob_state_.theta < -M_PI) rob_state_.theta += 2.0 * M_PI;
        }
        
        // 10. 调试输出
        static int odom_count = 0;
        if (++odom_count % 100 == 0 && odom_show_)
        {
            ROS_INFO("Odometry Update: x=%.3f, y=%.3f, theta=%.3f, vx=%.3f, vy=%.3f, omega=%.3f", 
                    rob_state_.x, rob_state_.y, rob_state_.theta, 
                    rob_state_.vx, rob_state_.vy, rob_state_.omega);
            odom_count = 0;
        }
    }

    void SentryChassisController::publishOdometry()
    {
        if (!publish_odom_ && !publish_tf_) return;

        ros::Time current_time = ros::Time::now();

        //创建odom消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        //设置位置
        odom.pose.pose.position.x = rob_state_.x;
        odom.pose.pose.position.y = rob_state_.y;
        odom.pose.pose.position.z = 0.0;

        //设置姿态(四元数)   
        tf2::Quaternion q;
        q.setRPY(0, 0, rob_state_.theta);
        geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
        odom.pose.pose.orientation = odom_quat;

        //设置协方差矩阵
        for (int i = 0; i < 36; i++)
        {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        
        //姿态协方差
        odom.pose.covariance[0] = 0.01;  // x位置方差
        odom.pose.covariance[7] = 0.01;  // y位置方差
        odom.pose.covariance[35] = 0.02; // 角度方差
        //速度协方差   
        odom.twist.covariance[0] = 0.01;  // x速度方差
        odom.twist.covariance[7] = 0.01;  // y速度方差
        odom.twist.covariance[35] = 0.02; // 角速度方差

        //发布odom消息
        if(publish_odom_) odom_pub_.publish(odom);

        //发布tf变换：odom -> base_link
        if (publish_tf_)
        {
            double dt = (current_time - last_tf_publish_time_).toSec();
            if (dt > tf_publish_period_)
            {
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = current_time;
                transformStamped.header.frame_id = odom_frame_id_;
                transformStamped.child_frame_id = base_frame_id_;
                
                transformStamped.transform.translation.x = rob_state_.x;
                transformStamped.transform.translation.y = rob_state_.y;
                transformStamped.transform.translation.z = 0.0;
                transformStamped.transform.rotation = odom_quat;

                //发布变换
                odom_broadcaster_.sendTransform(transformStamped);
                last_tf_publish_time_ = current_time;//更新最后发布里程计的时间
            }           
        }     
    }

    void SentryChassisController::initPowerManagement(ros::NodeHandle & controller_nh)
    {
        //读取参数
        power_mgt_.is_enable_ = controller_nh.param("enable_power_limit", true);
        power_mgt_.is_print_ = controller_nh.param("enable_power_print", true);
        power_mgt_.max_power_ = controller_nh.param("max_power", 360.0);
        power_mgt_.k1_ = controller_nh.param("k1", 0.001);
        power_mgt_.k2_ = controller_nh.param("k2", 0.0001);
        power_mgt_.torque_constant_ = controller_nh.param("torque_constant", 0.05);
        //初始化参数
        power_mgt_.current_power_ = 0.0;
        power_mgt_.last_power_update_ = ros::Time::now();

        if(!power_mgt_.is_enable_) return;
        if (power_mgt_.is_print_)
        {
            ROS_INFO("Power management initialized:");
            ROS_INFO("  Max power: %.1f W", power_mgt_.max_power_);
            ROS_INFO("  k1: %.6f, k2: %.6f", power_mgt_.k1_, power_mgt_.k2_);
            ROS_INFO("  Torque constant: %.3f Nm/A", power_mgt_.torque_constant_);
        }
        

    }

    double SentryChassisController::calculateTorqueLimitFactor()
    {
        if(!power_mgt_.is_enable_) return 1.0;
        //获取轮子转速
        double fl_vel = front_left_wheel_joint_.getVelocity();
        double fr_vel = front_right_wheel_joint_.getVelocity();
        double bl_vel = back_left_wheel_joint_.getVelocity();
        double br_vel = back_right_wheel_joint_.getVelocity();
        //获取力矩    
        double fl_tau = front_left_wheel_joint_.getEffort();
        double fr_tau = front_right_wheel_joint_.getEffort();
        double bl_tau = back_left_wheel_joint_.getEffort();
        double br_tau = back_right_wheel_joint_.getEffort();
        //计算总输出功率
        double P_out = fabs(fl_tau * fl_vel) + fabs(fr_tau * fr_vel) + fabs(bl_tau * bl_vel) + fabs(br_tau * br_vel);
        //计算损耗功率
        double tau_sq_sum = fl_tau*fl_tau + fr_tau*fr_tau + bl_tau*bl_tau + br_tau*br_tau;
        double omega_sq_sum = fl_vel*fl_vel + fr_vel*fr_vel + bl_vel*bl_vel + br_vel*br_vel;
        double P_loss = power_mgt_.k1_ * tau_sq_sum + power_mgt_.k2_ * omega_sq_sum;
        //输入总功率
        double P_in = P_out + P_loss;
        //更新当前功率
        power_mgt_.current_power_ = P_in;
        //没超过，不限制
        if(P_in <= power_mgt_.max_power_) return 1.0;
        //计算公式 3.3.4.4.2，计算缩放系数k
        double a = power_mgt_.k1_ * tau_sq_sum;
        double b = P_out;
        double c = power_mgt_.k2_ * omega_sq_sum - power_mgt_.max_power_;
        // 解一元二次方程，取合理正根
        double discriminant = b*b - 4*a*c;
        if(discriminant < 0) return 0.5; // 保护性限制
        double k1 = (-b + sqrt(discriminant)) / (2*a);
        double k2 = (-b - sqrt(discriminant)) / (2*a);
        double k = (k1 > 0 && k1 <= 1.0) ? k1 : ((k2 > 0 && k2 <= 1.0) ? k2 : 0.5);

        //调试输出
        static ros::Time last_caculate_time;
        if ((ros::Time::now() - last_caculate_time).toSec() > 1.0 && power_mgt_.is_print_) {
            ROS_INFO("Power limit: P_in=%.1fW, P_max=%.1fW, k=%.3f", P_in, power_mgt_.max_power_, k);
            last_caculate_time = ros::Time::now();
        }

        return std::max(0.3, std::min(1.0, k)); // 限制在0.3~1.0之间
    }

    void SentryChassisController::applyPowerLimiting()
    {
        if(!power_mgt_.is_enable_) return;
        static ros::Time last_apply_time;
        double limit_factor = calculateTorqueLimitFactor();
        if (limit_factor < 0.98)//显著限制时才应用
        {
            //等比例降低所有轮子速度
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] *= limit_factor;
            }
            if ((ros::Time::now() - last_apply_time).toSec() > 1.0 && power_mgt_.is_print_) 
            {
                ROS_WARN("Power limiting enabled: factor=%.2f, power=%.1fW", limit_factor, power_mgt_.current_power_);
                last_apply_time = ros::Time::now();
            }
        }
    }

    void SentryChassisController::printExpectedSpeed()
    {
        static ros::Time last_print_ = ros::Time::now();
        if ((ros::Time::now() - last_print_).toSec() > 1.0)
        {
            if(print_expected_speed_) 
            {    
                ROS_INFO("Speed expected > FL Wheel: %f, FR Wheel: %f, BL Wheel: %f, BR Wheel: %f", wheel_cmd_[0], wheel_cmd_[1], wheel_cmd_[2], wheel_cmd_[3]);
                ROS_INFO("Speed gotten   > FL Wheel: %f, FR Wheel: %f, BL Wheel: %f, BR Wheel: %f", front_left_wheel_joint_.getVelocity(), front_right_wheel_joint_.getVelocity(), back_left_wheel_joint_.getVelocity(), back_right_wheel_joint_.getVelocity());
                ROS_INFO("Difference     > FL Wheel: %f, FR Wheel: %f, BL Wheel: %f, BR Wheel: %f", wheel_cmd_[0] - front_left_wheel_joint_.getVelocity(), wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), wheel_cmd_[3] - back_right_wheel_joint_.getVelocity());
            }
            if(print_expected_pivot_) 
            {
                ROS_INFO("Pivot expected > FL: %f, FR: %f, BL: %f, BR: %f", pivot_cmd_[0], pivot_cmd_[1], pivot_cmd_[2], pivot_cmd_[3]);
                ROS_INFO("Pivot gotten   > FL: %f, FR: %f, BL: %f, BR: %f", front_left_pivot_joint_.getPosition(), front_right_pivot_joint_.getPosition(), back_left_pivot_joint_.getPosition(), back_right_pivot_joint_.getPosition());
                ROS_INFO("Difference     > FL: %f, FR: %f, BL: %f, BR: %f", pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), pivot_cmd_[3] - back_right_pivot_joint_.getPosition());
            }
        }
    }

    void SentryChassisController::starting(const ros::Time& time)
    {
        ROS_INFO("Sentry Chassis Controller started");
        reset_pid();
        last_update_time_ = time;
    }

    void SentryChassisController::stopping(const ros::Time& time){
        // 停止时发送零力矩
        front_left_pivot_joint_.setCommand(0.0);
        front_right_pivot_joint_.setCommand(0.0);
        back_left_pivot_joint_.setCommand(0.0);
        back_right_pivot_joint_.setCommand(0.0);
        front_left_wheel_joint_.setCommand(0.0);
        front_right_wheel_joint_.setCommand(0.0);
        back_left_wheel_joint_.setCommand(0.0);
        back_right_wheel_joint_.setCommand(0.0);
    }

    bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        //获取参数
        get_parameters(controller_nh);
        //初始化功率管理
        initPowerManagement(controller_nh);
        //初始化关节
        get_joint(effort_joint_interface, root_nh, controller_nh);
        //获取PID参数
        get_pid_parameters(controller_nh);
        //重置PID控制器
        reset_pid();
        //初始化PID控制器
        init_pid_parameters(controller_nh);
        //初始化功率管理
        initPowerManagement(controller_nh);

        dyn_reconf_server_ = std::make_shared<dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisConfig>>(controller_nh);
        dyn_reconf_callback_ = boost::bind(&SentryChassisController::dynamicReconfigureCallback, this, _1, _2);
        dyn_reconf_server_->setCallback(dyn_reconf_callback_);
        ROS_INFO("动态参数服务器已初始化");

        //初始化 twist消息
        gotten_msg.linear.x = 0;
        gotten_msg.linear.y = 0;
        gotten_msg.angular.z = 0;
        received_msg_ = false;
        //初始化订阅者
        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SentryChassisController::CmdVelCallback, this);
        last_angular_ = 0.0;
        last_update_time_ = ros::Time::now();
        //初始化关节状态发布者
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("controller/joint_states", 10);
        joint_names_ = {"left_front_pivot_joint", "right_front_pivot_joint", "left_back_pivot_joint", "right_back_pivot_joint",
                        "left_front_wheel_joint", "right_front_wheel_joint", "left_back_wheel_joint", "right_back_wheel_joint"};
        joint_state_msg_.name = joint_names_;
        joint_state_msg_.position.resize(joint_names_.size(), 0.0);
        joint_state_msg_.velocity.resize(joint_names_.size(), 0.0);
        joint_state_msg_.effort.resize(joint_names_.size(), 0.0);
        //初始化里程计发布者-相关参数在 get_parameters() 中获取
        last_tf_publish_time_ = ros::Time::now();
        odom_broadcaster_ = tf2_ros::TransformBroadcaster();
        if (publish_odom_) {
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
            ROS_INFO("Odometry publisher initialized on topic /odom");
        }
        //初始化机器人里程计状态
        rob_state_.x = 0.0;
        rob_state_.y = 0.0;
        rob_state_.theta = 0.0; 
        rob_state_.vx = 0.0;
        rob_state_.vy = 0.0;
        rob_state_.omega = 0.0;
        rob_state_.wheel_radius = 0.076; //假设轮子半径

        ROS_INFO("Sentry Chassis Controller initialized");
        ROS_INFO("Wheel track: %.3f m, Wheel base: %.3f m", wheel_track_, wheel_base_);
        ROS_INFO("Wheel radius: %.3f m", rob_state_.wheel_radius);
        ROS_INFO("Drive mode:%d   Turn mode:%d", drive_mode_, turn_mode_);
        ROS_INFO("max direction:%f", max_direction_);
        return true;
    }

    void SentryChassisController::update(const ros::Time &time, const ros::Duration &period)
    {
        //如果时间阈值内键盘没有输入正确的指令，就停止小车
        if ((time - last_update_time_).toSec() > stop_time_)
        {
            received_msg_ = false, last_update_time_ = time;
        }
        //先计算里程计，确保用到的robot_state_.theta是最新的
        calculateOdometry(period);
        //使用局部变量避免访问成员变量时的混乱
        double robot_vx = gotten_msg.linear.x;;
        double robot_vy = gotten_msg.linear.y;
        double robot_angular = gotten_msg.angular.z;
        //计算xy变换
        convertToRobotFrame(robot_vx, robot_vy, robot_angular);
        //计算轮子速度和转向角度
        applyAccelerationLimits(robot_vx, robot_vy, robot_angular, period);
        calculateWheelCommands(robot_vx, robot_vy,robot_angular);
        //应用功率限制
        applyPowerLimiting();
        //设置关节(8个)
        setSpeedPivot(period);
        //发布里程计
        publishOdometry();
        //发布关节状态
        publishJointStates();
    }
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
