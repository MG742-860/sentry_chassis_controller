#ifndef _SENTRY_CHASSIS_CONTROLLER_
#define _SENTRY_CHASSIS_CONTROLLER_
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <memory>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Dense>
#include <Eigen/QR>
//#include "sentry_chassis_controller/SentryChassisConfig.h"
#include "../../../devel/include/sentry_chassis_controller/SentryChassisConfig.h"
namespace sentry_chassis_controller{

    enum DriveMode{
        ForwardDrive = 0,//前驱
        BackwardDrive = 1,//后驱
        AllDrive = 2//四驱
    };

    enum TurnMode{
        ForwardTurn = 0,//前转
        BackwardTurn = 1,//后转
        AllTurn = 2,//全转
        NoneTurn =3//履带-不转
    };

    struct wheel_state {
        double vx_ = 0;
        double vy_ = 0;
        double x_ = 0;
        double y_ = 0;
        double direction_ = 0;
        double direction_m_ = 0;
        double speed_ = 0;
    };

    struct rob_state{
        double x;           // x位置 (m)
        double y;           // y位置 (m)
        double theta;       // 朝向 (rad)
        double vx;          // x方向速度 (m/s)
        double vy;          // y方向速度 (m/s)
        double omega;       // 角速度 (rad/s)
        double wheel_radius; // 轮子半径 (m)
    };
    
    struct PowerManagement {
        double k1_ = 0.001;              //力矩损耗系数
        double k2_ = 0.0001;             //转速损耗系数
        double max_power_ = 0;           //最大允许功率 (W)
        double current_power_ = 0;       //当前功率 (W)
        double torque_constant_ = 0.05;   //电机转矩常数 (Nm/A)
        bool is_enable_ = false;         //是否启用功率限制
        bool is_print_ = true;           //是否打印调试信息
        ros::Time last_power_update_;    //上一次更新时间

        //缓存计算
        double total_torque_sq_ = 0.0;
        double total_omega_sq_ = 0.0;
        double total_torque_omega_ = 0.0;
    };

    struct MotionLimits {
        bool is_enable = false;
        double max_linear_acceleration = 3.0;   // 最大线加速度 (m/s²)
        double max_angular_acceleration = 5.0;  // 最大角加速度 (rad/s²)
        double last_vx_ = 0.0, last_vy_ = 0.0, last_omega_ = 0.0;//上一次的数据
    };


    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
        public:
            //默认构造函数和析构函数
            SentryChassisController() = default;
            ~SentryChassisController() override = default;

            //获取并设置关节句柄和参数
            void get_joint(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
            void get_parameters(ros::NodeHandle &controller_nh);
            void get_pid_parameters(ros::NodeHandle &controller_nh);
            void init_pid_parameters(ros::NodeHandle &controller_nh);
            void setSpeedPivot(const ros::Duration &period);
            void reset_pid();
            void dynamicReconfigureCallback(sentry_chassis_controller::SentryChassisConfig &config, uint32_t level);

            //发布关节状态
            void publishJointStates();

            //回调函数
            void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
            void convertToRobotFrame(double& robot_x, double& robot_y, double& robot_angular);//(轮子)坐标变换函数 odom -> base_link
            //计算轮子速度和转向角度
            void calculateWheel(double direction, double speed, wheel_state wheel_state_[]);
            void calculatePivot(double direction,double angular, wheel_state wheel_state_[]);
            void applyAccelerationLimits(double& vx_target, double& vy_target, double& omega_target, const ros::Duration& period);
            void calculateWheelCommands(double vx, double vy,double angular);
            void handleDifferentialSteering(double angular);  //处理差速转向

            //里程计相关函数
            void calculateOdometry(const ros::Duration& period);
            void publishOdometry();
            
            //功率限制相关
            void initPowerManagement(ros::NodeHandle &controller_nh);
            double calculateTorqueLimitFactor();
            void applyPowerLimiting();

            //打印测试函数
            void printExpectedSpeed();

            //重写基类函数
            void starting(const ros::Time& time) override;
            void stopping(const ros::Time& time) override;
            bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
            void update(const ros::Time &time, const ros::Duration &period) override;
        private:
            //关节句柄类
            hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;
            double wheel_cmd_[4], pivot_cmd_[4];
            double wheel_track_, wheel_base_, wheel_radius_;
            //关节发布
            ros::Publisher joint_state_pub_;
            sensor_msgs::JointState joint_state_msg_;
            std::vector<std::string> joint_names_;
   
            //PID控制器
            control_toolbox::Pid pid_fl_pivot_, pid_fr_pivot_, pid_bl_pivot_, pid_br_pivot_;
            control_toolbox::Pid pid_fl_wheel_, pid_fr_wheel_, pid_bl_wheel_, pid_br_wheel_;
            control_toolbox::Pid::Gains gains_pivot_, gains_wheel_;
            std::shared_ptr<dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisConfig>> dyn_reconf_server_;
            dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisConfig>::CallbackType dyn_reconf_callback_;
            wheel_state wheel_state_[4];
            MotionLimits motion_limits_;

            // /cmd_vel 相关
            ros::NodeHandle nh_;
            geometry_msgs::Twist gotten_msg;
            double last_angular_;
            ros::Subscriber cmd_vel_sub_;
            bool received_msg_;
            ros::Time last_update_time_;
            double stop_time_;

            // /odom 相关
            ros::Publisher odom_pub_;
            tf2_ros::TransformBroadcaster odom_broadcaster_;
            ros::Time last_tf_publish_time_;
            double tf_publish_period_;
            rob_state rob_state_;
            bool publish_tf_, publish_odom_;
            std::string odom_frame_id_, base_frame_id_;

            //驱动和转向模式
            DriveMode drive_mode_;
            TurnMode turn_mode_;

            //功率参数
            PowerManagement power_mgt_;

            //其他参数
            int speed_to_rotate_;
            double speed_diff_m_;
            bool print_expected_speed_, print_expected_pivot_, odom_show_;
            double max_speed_, max_angular_, max_direction_;
            bool coordinate_mode_;  // false: 底盘坐标系，true: 全局坐标系

            //当前小车状态-计算状态
            double current_speed_;//当前线速度
            double current_direction_;//当前运动方向(角度)
            double current_angular_;//当前角速度
    };
}
#endif