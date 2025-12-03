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
namespace sentry_chassis_controller{

    struct rob_state{
        double x;           // x位置 (m)
        double y;           // y位置 (m)
        double theta;       // 朝向 (rad)
        double vx;          // x方向速度 (m/s)
        double vy;          // y方向速度 (m/s)
        double omega;       // 角速度 (rad/s)
        double wheel_radius; // 轮子半径 (m)
    };
    
    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
        public:
            SentryChassisController() = default;
            ~SentryChassisController() override = default;
            void starting(const ros::Time& time) override;
            void stopping(const ros::Time& time) override;
            bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
            void update(const ros::Time &time, const ros::Duration &period) override;
            hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;
            void setSpeedPivot(const ros::Duration &period);
            void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
            void calculateWheelCommands(double vx, double vy,double angular);
            void printExpectedSpeed();
            void calculateOdometry(const ros::Duration& period);
            void publishOdometry();
            void publishJointStates();
        private:
            ros::Time last_time_;
            double stop_time_;
            double wheel_track_, wheel_base_;
            double pivot_cmd_[4];
            double wheel_cmd_[4];
            control_toolbox::Pid pid_fl_pivot_, pid_fr_pivot_, pid_bl_pivot_, pid_br_pivot_;
            control_toolbox::Pid pid_fl_wheel_, pid_fr_wheel_, pid_bl_wheel_, pid_br_wheel_;
            control_toolbox::Pid::Gains gains_pivot_, gains_wheel_;
            //订阅 /cmd_vel 相关
            ros::NodeHandle nh_;
            geometry_msgs::Twist gotten_msg;
            double last_angular_;
            ros::Subscriber cmd_vel_sub_;
            bool received_msg_;
            //订阅 /odom 相关
            ros::Publisher odom_pub_;
            tf2_ros::TransformBroadcaster odom_broadcaster_;
            ros::Time last_tf_publish_time_;
            double tf_publish_period_;
            rob_state rob_state_;
            bool publish_tf_, publish_odom_;
            std::string odom_frame_id_, base_frame_id_;

            //关节发布
            ros::Publisher joint_state_pub_;
            sensor_msgs::JointState joint_state_msg_;
            std::vector<std::string> joint_names_;

            //其他参数
            int speed_to_rotate_;
            bool print_expected_speed_, print_expected_pivot_, odom_show_;
            bool is_forward_lock_;
            double max_speed_, max_angular_;
    };
}
#endif