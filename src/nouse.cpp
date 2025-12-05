/*
                is_forward_lock_ = controller_nh.param("is_forward_lock", false);
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
            //ROS_INFO("Adding angular velocity: %f", angular);
            wheel_cmd_[0] -= angular;
            wheel_cmd_[2] -= angular;
            wheel_cmd_[1] += angular;
            wheel_cmd_[3] += angular;
        }
        printExpectedSpeed();

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
*/