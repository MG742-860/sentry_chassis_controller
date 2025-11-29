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