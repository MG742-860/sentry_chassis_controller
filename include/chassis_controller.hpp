#ifndef _CHASSIS_CONTROLLER_
#define _CHASSIS_CONTROLLER_
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

namespace chassis_controller {
class ChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    ChassisController() = default;
    ~ChassisController() override = default;
    
    bool init(hardware_interface::EffortJointInterface *effort_joint_interface, 
              ros::NodeHandle &root_nh, 
              ros::NodeHandle &controller_nh) override;
    
    void update(const ros::Time &time, const ros::Duration &period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

    // 新增：必须实现的ControllerBase接口
    // void update(const ros::Time& time, const ros::Duration& period, 
    //             hardware_interface::RobotHW* robot_hw = nullptr) override {
    //     update(time, period); // 调用我们自己的update函数
    // }

    // 工具函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void calculateWheelCommands(double vx, double vy, double omega);
    void updateGains(double kp, double ki, double kd, double i_max, double i_min, bool is_wheel);
    void setSpeed(double speed);

    // 关节句柄
    hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, 
                                   back_left_pivot_joint_, back_right_pivot_joint_;
    hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, 
                                   back_left_wheel_joint_, back_right_wheel_joint_;

private:
    geometry_msgs::Twist current_cmd_;
    std::mutex cmd_mutex_;
    ros::Time last_cmd_time_;
    bool received_cmd_;

    double wheel_track_;
    double wheel_base_;
    double pivot_cmd_[4];
    double wheel_cmd_[4];
    double i_max_, i_min_, i_max_w_, i_min_w_, speed_;

    // PID增益和控制器
    struct Gains {
        double p_gain_ = 1.0;
        double i_gain_ = 0.0;
        double d_gain_ = 0.0;
    };
    
    Gains gains_, gains_wheel_;
    control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
    control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;

    ros::Subscriber cmd_vel_sub_;
    ros::NodeHandle nh_;
};
}
#endif