# 哨兵底盘控制器
### 使用说明
	本控制器自带键盘控制器，在启动文件中自行设置是否启动，如果有其他键盘控制器可以用，也可以不启动自带的键盘控制器，使用你自己的键盘控制器。
在使用本控制器之前，你需要下载并编译以下包：
* **rm_control**
* **rm_description(rm_description_for_task)**

本控制器有四种转向方式：
	**前转  后转  全转  不转(履带)**
以及三种驱动方式：
	**前驱  后驱  全驱**
最后还有两种坐标系选择： 
	**底盘坐标系 全局坐标系**
**注意：**
* 全转：
	全转一定是全驱，不管你怎样设置，只要启用了全转，那么一定是全驱
* 不转：
	指像坦克那样通过左右差速进行转向，转向差速与angular有关，如果转向效果差，请增加键盘控制器发布的angular的值

此外，本控制器还有功率限制模块，可以在配置文件中选择启用/禁用。

如果你对本控制器没有任何疑问等，在编译之后使用命令：
```bash
# catkin_make 编译命令
roslaunch sentry_chassis_controller run_controller.launch
```
然后就可以启动gazebo和键盘控制器了。
在run_controller.launch文件中你也可以更改true/false值来启用/禁用某些节点，比如键盘控制器和rviz，请自行更改
***
**所有驱动转向组合均经过测试，理论运动情况符合预期，但是gazebo中为理想条件，以及pid参数不同会导致打滑等情况，可自行添加gazebo摩擦系数、调整pid进行解决**
有关功率限制：
	功率限制现阶段无法获取电流，只能通过扭矩计算出电流：
	``` 扭矩 = 电流 * 扭转系数 ```
	扭转系数自行在配置文件中修改
### 文件说明
***
**两个主要文件：**
* 哨兵底盘控制器插件：
	* sentry_chassis_controller.cpp - 源码源文件
	* sentry_chassis_controller.hpp - 源码头文件
	* controller.yaml - 用于设置参数，控制驱动、转向模式等
	用于接收键盘控制器的信息控制底盘
* 键盘控制器文件：
	* teleop_twist_keyboard.cpp - 源码源文件
	* teleop_twist_keyboard.hpp - 源码头文件
	* teleop_twist_keyboard.yaml - 键盘控制器参数，设置键位、初始速度等。
	用于发布/cmd_vel话题消息 
***
详细文件结构：
* **sentry_chassis_controller**
	* config
		* controller.yaml
		* teleop_twist_keyboard.yaml
		* sentry.rviz 
	* include
		* sentry_chassis_controller.hpp
		* teleop_twist_keyboard.hpp
	* src
		* sentry_chassis_controller.cpp
		* teleop_twist_keyboard.cpp
	* launch
		* load_controller.launch
		* run_controller.launch
	* chassis_controller_plugins.xml
	* CMakeLists.txt
	* package.xml
	* README.md
	* update.md