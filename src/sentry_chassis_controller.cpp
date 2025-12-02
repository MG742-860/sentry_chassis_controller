#include "../include/sentry_chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace sentry_chassis_controller {
    bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        //获取打印参数
        print_expected_speed_ = controller_nh.param("print_expected_speed", false);
        print_expected_pivot_ = controller_nh.param("print_expected_pivot", false);
        
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
        speed_to_rotate_ = controller_nh.param("speed_to_rotate", 100);
        //初始化停止时间阈值
        stop_time_ = controller_nh.param("stop_time", 0.5);
        //初始化 twist消息
        gotten_msg.linear.x = 0;
        gotten_msg.linear.y = 0;
        gotten_msg.angular.z = 0;
        received_msg_ = false;

        //初始化订阅者
        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SentryChassisController::CmdVelCallback, this);
        last_angular_ = 0.0;
        last_time_ = ros::Time::now();

        //初始化轮子信息
        wheel_base_ = controller_nh.param("wheel_base", 0.5);//前后轮间距
        wheel_track_ = controller_nh.param("wheel_track", 0.5);//左右轮间距

        //初始化里程计发布者
        last_tf_publish_time_ = ros::Time::now();
        tf_publish_period_ = controller_nh.param("tf_publish_period", 0.05);
        publish_odom_ = controller_nh.param("publish_odom", true);
        publish_tf_ = controller_nh.param("publish_tf", true);
        odom_frame_id_ = controller_nh.param("odom_frame_id", std::string("odom"));
        base_frame_id_ = controller_nh.param("base_frame_id", std::string("base_link"));
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
        return true;
    }

    void SentryChassisController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        gotten_msg = *msg;
        last_angular_ = gotten_msg.angular.z ? gotten_msg.angular.z : last_angular_;
        received_msg_ = true;
        last_time_ = ros::Time::now();
    }

    void SentryChassisController::printExpectedSpeed()
    {
        if(print_expected_pivot_ || print_expected_speed_) ROS_INFO("----------------------------------------\nis forward locked: %s", is_forward_lock_ ? "true" : "false");
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

    void SentryChassisController::calculateWheelCommands(double vx, double vy, double angular)
    {
        //先判断键盘是否有消息
        if (!received_msg_)
        {
            //如果没有，就自锁小车
            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = 0;
            }
                pivot_cmd_[0] = -M_PI_2 * 0.5;
                pivot_cmd_[1] = -M_PI * 0.75;
                pivot_cmd_[2] = M_PI_2 * 0.5;
                pivot_cmd_[3] = M_PI * 0.75;
                //不打印预期值，避免终端信息污染
            return;
        }
        //如果有消息，判断是否超过设定的最大值
        if (abs(vx) > abs(max_speed_))
        {
            vx = max_speed_ * (abs(vx) / vx);
        }
        if (abs(vy) > abs(max_speed_))
        {
            vy = max_speed_ * (abs(vy) / vy);
        }
        if (abs(angular) > abs(max_angular_))
        {
            angular = max_angular_ * (abs(angular) / angular);
        }
                

        if (is_forward_lock_)
        {
            //开始计算角度
            if (angular && !atan2(vy, vx))
            {
                //如果有角速度但是没有线速度，就原地转圈
                for (int i = 0; i < 4; i++)
                {
                    wheel_cmd_[i] = speed_to_rotate_ * angular;
                }
                pivot_cmd_[0] = -M_PI_2 * 0.5;
                pivot_cmd_[1] = -M_PI * 0.75;
                pivot_cmd_[2] = M_PI_2 * 0.5;
                pivot_cmd_[3] = M_PI * 0.75;
                printExpectedSpeed();
                return;
            }
            
            angular = atan2(vy, vx);
            double speed = sqrt(vx * vx + vy * vy);

            for (int i = 0; i < 4; i++)
            {
                wheel_cmd_[i] = speed;
                pivot_cmd_[i] = angular;
            }
            printExpectedSpeed();
            return;
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
        if (vy && !vx)
        {
            angular = last_angular_ * (abs(vy) / vy);
        }
        
        //然后加权后设置转向速度
        if (angular)
        {
            wheel_cmd_[0] -= angular;
            wheel_cmd_[2] -= angular;
            wheel_cmd_[1] += angular;
            wheel_cmd_[3] += angular;
        }
        printExpectedSpeed();
    }

    void SentryChassisController::calculateOdometry(const ros::Duration& period)
    {
        // 获取实际轮子速度（m/s）
        double fl_wheel_vel = front_left_wheel_joint_.getVelocity();
        double fr_wheel_vel = front_right_wheel_joint_.getVelocity();
        double bl_wheel_vel = back_left_wheel_joint_.getVelocity();
        double br_wheel_vel = back_right_wheel_joint_.getVelocity();
        
        // 获取实际转向角度
        double fl_pivot_angle = front_left_pivot_joint_.getPosition();
        double fr_pivot_angle = front_right_pivot_joint_.getPosition();
        double bl_pivot_angle = back_left_pivot_joint_.getPosition();
        double br_pivot_angle = back_right_pivot_joint_.getPosition();

        if(is_forward_lock_)
        {
            // 全向移动模式：根据实际轮子速度和角度计算底盘速度
            // 这里使用简化的方法：取四个轮子速度的平均作为底盘速度
            rob_state_.vx = (fl_wheel_vel * cos(fl_pivot_angle) + fr_wheel_vel * cos(fr_pivot_angle) + bl_wheel_vel * cos(bl_pivot_angle) + br_wheel_vel * cos(br_pivot_angle)) / 4.0;         
            rob_state_.vy = (fl_wheel_vel * sin(fl_pivot_angle) + fr_wheel_vel * sin(fr_pivot_angle) + bl_wheel_vel * sin(bl_pivot_angle) + br_wheel_vel * sin(br_pivot_angle)) / 4.0;
            // 计算角速度：使用差速模型
            // 对于万向轮，角速度计算比较复杂，这里使用简化方法
            double avg_pivot_angle = (fl_pivot_angle + fr_pivot_angle + bl_pivot_angle + br_pivot_angle) / 4.0;
            rob_state_.omega = (avg_pivot_angle - rob_state_.theta) / period.toSec();
        }
        else
        {
            // 差速移动模式：根据左右轮速度差计算底盘速度和角速度
            double left_wheel_vel = (fl_wheel_vel + bl_wheel_vel) / 2.0;
            double right_wheel_vel = (fr_wheel_vel + br_wheel_vel) / 2.0;

            rob_state_.vx = (left_wheel_vel + right_wheel_vel) / 2.0;
            rob_state_.vy = 0.0; // 差速模式下没有侧向速度
            //omega计算: omega = (v_right - v_left) / wheel_track
            rob_state_.omega = (right_wheel_vel - left_wheel_vel) / wheel_track_;
        }

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
        if (++odom_count % 100 == 0) // 每100次打印一次
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
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(rob_state_.theta);
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
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(rob_state_.x, rob_state_.y, 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, rob_state_.theta);
                transform.setRotation(q);
                odom_broadcaster_.sendTransform(tf::StampedTransform(transform, current_time, odom_frame_id_, base_frame_id_));
            }
            last_tf_publish_time_ = current_time;//更新最后发布里程计的时间
        }
        
    }

    void SentryChassisController::setSpeedPivot(const ros::Duration &period)
    {
        front_left_wheel_joint_.setCommand(pid_fl_wheel_.computeCommand(wheel_cmd_[0]/*6*/ - front_left_wheel_joint_.getVelocity(), period));
        front_right_wheel_joint_.setCommand(pid_fr_wheel_.computeCommand(wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), period));
        back_left_wheel_joint_.setCommand(pid_bl_wheel_.computeCommand(wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), period));
        back_right_wheel_joint_.setCommand(pid_br_wheel_.computeCommand(wheel_cmd_[3] - back_right_wheel_joint_.getVelocity(), period));

        front_left_pivot_joint_.setCommand(pid_fl_pivot_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period));
        front_right_pivot_joint_.setCommand(pid_fr_pivot_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period));
        back_left_pivot_joint_.setCommand(pid_bl_pivot_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period));
        back_right_pivot_joint_.setCommand(pid_br_pivot_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period));
    }

    void SentryChassisController::update(const ros::Time &time, const ros::Duration &period)
    {
        //如果时间阈值内键盘没有输入正确的指令，就停止小车
        if ((time - last_time_).toSec() > stop_time_)
        {
            received_msg_ = false, last_time_ = time;
        }
        //计算轮子速度和转向角度
        calculateWheelCommands(gotten_msg.linear.x, gotten_msg.linear.y,gotten_msg.angular.z);
        setSpeedPivot(period);
        //计算并发布里程计
        calculateOdometry(period);
        publishOdometry();
    }
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
