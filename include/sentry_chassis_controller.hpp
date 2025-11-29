#ifndef _SENTRY_CHASSIS_CONTROLLER_
#define _SENTRY_CHASSIS_CONTROLLER_
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
namespace sentry_chassis_controller{

    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
        public:
            SentryChassisController() = default;
            ~SentryChassisController() override = default;
            bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
            void update(const ros::Time &time, const ros::Duration &period) override;
            hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;
            void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
            void calculateWheelCommands(double vx, double vy, double angular, bool is_lock);
        private:
            ros::Time time;
            double wheel_track_;
            double wheel_base_;
            double pivot_cmd_[4];
            double wheel_cmd_[4];
            control_toolbox::Pid pid_fl_pivot_, pid_fr_pivot_, pid_bl_pivot_, pid_br_pivot_;
            control_toolbox::Pid pid_fl_wheel_, pid_fr_wheel_, pid_bl_wheel_, pid_br_wheel_;
            control_toolbox::Pid::Gains gains_pivot_, gains_wheel_;
            //订阅 /cmd_vel 相关
            ros::NodeHandle nh_;
            geometry_msgs::Twist gotten_msg;
            ros::Subscriber cmd_vel_sub_;
            bool received_msg_;

            bool is_lock_;
            double max_speed_;
    };
}
#endif