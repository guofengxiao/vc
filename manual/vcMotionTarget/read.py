vcMotionTarget
"vcMotionTarget provides properties and methods for defining how a robot moves. Such movements may include joint and tool behavior as well as speed and configuration settings for orientation solutions—see Articulated Kinematics for more information."

总结：
01. vcMotionTarget没有加速度这个选项
定义机器人怎样运动
运动包括：与速度和状态设定的关节行为和工具行为

问题1：vcRobotController.move()时间与机器人在Teach时间不一致
问题2：从statement读取的vcMotionTarget 仿真move不了。



01. createTarget 被创建
  vcRobotController.createTarget()
  vcRobotController.createTarget( vcMatrix )
    区别: 相对当前机器人base的坐标位置区别
    
 02. 作为参数
   a. addTarget
     vcRobotController.addTarget( vcMotionTarget )
     vcRobotController.addTarget( vcMatrix )