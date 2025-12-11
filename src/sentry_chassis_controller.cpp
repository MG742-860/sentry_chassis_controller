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
        //速度、角度、转角最大值限制
        max_speed_ = controller_nh.param("max_speed", 1.0);
        max_angular_ = controller_nh.param("max_angular", 1.0);
        double max_direction_c = controller_nh.param("max_direction", 90.0);
        max_direction_ = max_direction_c * M_PI / 180.0; // 转化为弧度
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
        ROS_INFO("动态参数更新 (level: %d)", level);
        
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
        stop_time_ = config.stop_time;
        
        // 更新功率参数
        power_mgt_.max_power_ = config.max_power;
        power_mgt_.is_enable_ = config.enable_power_limit;
        
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

    void SentryChassisController::calculateWheel(const double direction, const double speed_src)
    {
        double speed = speed_src / wheel_radius_;//目标速度转化为轮子转速
        // 根据驱动模式调整轮子速度
        switch (drive_mode_)
        {
            case ForwardDrive://前驱
                wheel_cmd_[0] += speed;//左前
                wheel_cmd_[1] += speed;//右前
                wheel_cmd_[2] = 0;//左后
                wheel_cmd_[3] = 0;//右后
                break;
            case BackwardDrive://后驱
                wheel_cmd_[0] = 0;//左前
                wheel_cmd_[1] = 0;//右前
                wheel_cmd_[2] += speed;//左后
                wheel_cmd_[3] += speed;//右后
                break;
            case AllDrive://全驱(默认)
            default:
                for (int i = 0; i < 4; i++)
                {
                    wheel_cmd_[i] += (turn_mode_ == AllTurn)?fabs(speed):speed;
                }
                break;
        }
    }

    void SentryChassisController::calculatePivot(const double direction_src, const double angular)//传入的是方向角、角速度
    {
        double direction_limit = direction_src;
        // 根据转向模式调整轮子转向角度
        switch (turn_mode_)
        {
            case ForwardTurn://前轮转向
                if (fabs(direction_limit) >= M_PI_2 - 1e-6)
                {
                    direction_limit = M_PI*(direction_limit/fabs(direction_limit)) - direction_limit;
                    direction_limit = fabs(max_direction_)<fabs(direction_limit)?(max_direction_)*(direction_limit/fabs(direction_limit)):direction_limit;
                }
                pivot_cmd_[0] = direction_limit;//左前
                pivot_cmd_[1] = direction_limit;//右前
                pivot_cmd_[2] = 0;//左后
                pivot_cmd_[3] = 0;//右后
                break;
            case BackwardTurn://后轮转向
                if (fabs(direction_limit) >= M_PI_2 - 1e-6)
                {
                    direction_limit = M_PI*(direction_limit/fabs(direction_limit)) - direction_limit;
                    direction_limit = fabs(max_direction_)<fabs(direction_limit)?(max_direction_)*(direction_limit/fabs(direction_limit)):direction_limit;
                }
                pivot_cmd_[0] = 0;//左前
                pivot_cmd_[1] = 0;//右前
                pivot_cmd_[2] = -direction_limit;//左后
                pivot_cmd_[3] = -direction_limit;//右后
                break;
            case AllTurn://全轮转向(默认)
                for (int i = 0; i < 4; i++)
                {
                    pivot_cmd_[i] = direction_src;
                }         
                break;
            case NoneTurn://不转向
                for (int i = 0; i < 4; i++)
                {
                    pivot_cmd_[i] = 0;
                }
                //然后应用差速转向(修改轮子速度)
                handleDifferentialSteering(angular);
                break;
            default:
                // 不做任何限制
                break;
        }
    }

    void SentryChassisController::handleDifferentialSteering(double angular)
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
                wheel_cmd_[0] -= speed_diff;  // 左前轮减速
                wheel_cmd_[1] += speed_diff;  // 右前轮加速
                break;
                
            case BackwardDrive:  // 后驱差速
                wheel_cmd_[2] -= speed_diff;  // 左后轮减速
                wheel_cmd_[3] += speed_diff;  // 右后轮加速
                break;
                
            case AllDrive:  // 四驱差速
            default:
                wheel_cmd_[0] -= speed_diff;  // 左前轮减速
                wheel_cmd_[1] += speed_diff;  // 右前轮加速
                wheel_cmd_[2] -= speed_diff;  // 左后轮减速
                wheel_cmd_[3] += speed_diff;  // 右后轮加速
                break;
        }
    }

    void SentryChassisController::calculateWheelCommands(double vx, double vy, double angular)
    {
        //先判断键盘是否有消息
        if (!received_msg_)
        {
            //如果没有，就自锁小车
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = 0;//速度归零
            }
                //轮子角度互交
                pivot_cmd_[0] = -M_PI_2 * 0.5;
                pivot_cmd_[1] = -M_PI * 0.75;
                pivot_cmd_[2] = M_PI_2 * 0.5;
                pivot_cmd_[3] = M_PI * 0.75;
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
                wheel_cmd_[i] = -(angular * speed_to_rotate_);
            }
            //轮子角度互交
            pivot_cmd_[0] = -M_PI_2 * 0.5;
            pivot_cmd_[1] = -M_PI * 0.75;
            pivot_cmd_[2] = M_PI_2 * 0.5;
            pivot_cmd_[3] = M_PI * 0.75;
            printExpectedSpeed();
            return;
        }

        //计算方向和速度
        double direction = atan2(vy, vx);
        double speed = sqrt(vx * vx + vy * vy) * (fabs(direction) > M_PI_2 ? -1:1);
        //先将数据置零
        for (int i = 0; i < 4; i++)
        {
            pivot_cmd_[i] = 0;
            wheel_cmd_[i] = 0;
        }
        calculatePivot(direction, angular);
        calculateWheel(direction, speed);
        printExpectedSpeed();
        return;
    }

    void SentryChassisController::calculateOdometry(const ros::Duration& period)
    {
        // 获取实际轮子速度(m/s)
        double fl_wheel_vel = front_left_wheel_joint_.getVelocity();
        double fr_wheel_vel = front_right_wheel_joint_.getVelocity();
        double bl_wheel_vel = back_left_wheel_joint_.getVelocity();
        double br_wheel_vel = back_right_wheel_joint_.getVelocity();
        
        // 获取实际转向角度
        double fl_pivot_angle = front_left_pivot_joint_.getPosition();
        double fr_pivot_angle = front_right_pivot_joint_.getPosition();
        double bl_pivot_angle = back_left_pivot_joint_.getPosition();
        double br_pivot_angle = back_right_pivot_joint_.getPosition();
   
        // 全向移动模式：根据实际轮子速度和角度计算底盘速度
        rob_state_.vx = (fl_wheel_vel * cos(fl_pivot_angle) + fr_wheel_vel * cos(fr_pivot_angle) + bl_wheel_vel * cos(bl_pivot_angle) + br_wheel_vel * cos(br_pivot_angle)) / 4.0;         
        rob_state_.vy = (fl_wheel_vel * sin(fl_pivot_angle) + fr_wheel_vel * sin(fr_pivot_angle) + bl_wheel_vel * sin(bl_pivot_angle) + br_wheel_vel * sin(br_pivot_angle)) / 4.0;
        // 计算角速度：使用差速模型
        // 对于万向轮，角速度计算比较复杂，这里使用简化方法
        double avg_pivot_angle = (fl_pivot_angle + fr_pivot_angle + bl_pivot_angle + br_pivot_angle) / 4.0;
        rob_state_.omega = (avg_pivot_angle - rob_state_.theta) / period.toSec();

        //限制速度范围，避免发生异常
        const double MAX_VELOCITY = max_speed_; // 最大线速度 (m/s)
        const double MAX_OMEGA = 5.0; // 最大角速度 (rad/s)
        rob_state_.vx = std::max(std::min(rob_state_.vx, MAX_VELOCITY), -MAX_VELOCITY);
        rob_state_.vy = std::max(std::min(rob_state_.vy, MAX_VELOCITY), -MAX_VELOCITY);
        rob_state_.omega = std::max(std::min(rob_state_.omega, MAX_OMEGA), -MAX_OMEGA);

        //更新机器人位置和姿态(积分)
        //将机体坐标系下的速度转换到全局坐标系下
        double delta_x = (rob_state_.vx * cos(rob_state_.theta) - rob_state_.vy * sin(rob_state_.theta)) * period.toSec();
        double delta_y = (rob_state_.vx * sin(rob_state_.theta) + rob_state_.vy * cos(rob_state_.theta)) * period.toSec();
        double delta_theta = rob_state_.omega * period.toSec();

        rob_state_.x += delta_x, rob_state_.y += delta_y, rob_state_.theta += delta_theta;//更新位置和姿态

        // 归一化theta到[-pi, pi]
        while (rob_state_.theta > M_PI) rob_state_.theta -= 2.0 * M_PI;
        while (rob_state_.theta < -M_PI) rob_state_.theta += 2.0 * M_PI;
        
        //调试输出
        static int odom_count = 0;
        if (++odom_count % 100 == 0 && odom_show_) // 每100次打印一次
        {
            ROS_INFO("Odometry Update: x=%.3f, y=%.3f, theta=%.3f, vx=%.3f, vy=%.3f, omega=%.3f", rob_state_.x, rob_state_.y, rob_state_.theta, rob_state_.vx, rob_state_.vy, rob_state_.omega);
            odom_count = 0;// 打印后重置计数器
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
