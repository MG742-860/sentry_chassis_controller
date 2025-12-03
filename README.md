# Sentry Chassis Controller of final work
***
READMD.md 记录一些有用的提交，不记录无用的提交，比如修改READMD.md
***
##### 第一次提交
log:
添加了初代源码，让小车可以订阅/cmd_vel话题消息，但是小车会抽搐，且对接收到的消息不做处理

hash: ada0f6e8152dd807e12bc02079e573b84a7bc553

branch: (HEAD -> master, origin/master)

date: 2025.11.28 12:01

##### 第二次提交
log:修复了一些bug，更改了运动计算模型，但是还是无法响应/cmd_vel的消息

hash：de5198ad2a8420bd5f47f63ee0cb6f6060051e29

branch：(HEAD -> master, origin/master)

date：2025.11.28 16：42

##### 第三次提交
log：
重新写了一份sentry_chassis_controller，抛弃了原来的chassis_controller，现在可以响应/cmd_vel的消息，可以正常使用，但是小车很难操控旋转

hash：54c74d09ea925bd87ab3a685e8db288c28e56254

branch：(HEAD -> master, origin/master)

date：2025.11.29 13：30

##### 第四次提交
log：
补全了两种运动模式，并且写了一份CPP版本的键盘控制器，添加了键盘服务器参数，可以正常控制小车运动，小车正常旋转。
(md为什么用了我一晚上)

hash：2e222d84074413cfaf204c0ebb03753b9a62822b

branch：(HEAD -> master, origin/master)

date：2025.12.1 22：26

##### 第五次提交
log：
键盘控制器添加了单独发送angular指令的键位，用于实现小陀螺，还在终端打印了发布速度、角度（通过cmd_vel解算后发布的轮子速度与角度，通过逆运动学）。
发布速度、角度时可以通过配置参数进行选择，发布的消息包括期望值、实际值和差值。
添加了正运动学计算实际速度与已经走过的路程(期望值)，添加了正运动学发布里程计（存在bug）

hash：5e2fd7f535fec5907ff762f8b558ace30c4f5afc

branch：(HEAD -> master, origin/master)

date：2025.12.2 23：40

##### 第六次提交
log：
修复了正运动学发布里程计bug，现在可以在rviz中正常显示，修复了键盘控制器执行小陀螺后角度数值过大（M_PI/4）的bug，更新了部分文件参数。

hash：

branch：(HEAD -> master, origin/master)

date：2025.12.3 23：17