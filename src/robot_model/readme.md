20210106-10:32
各个包的功能如下：
1. real_mobile_robot: 包含N_robot、N_robot_topic以及robot_base_odometry，其中前两个包与实际机器人通讯，后一个包发布实际机器人的里程计信息，根据实际需求可以进行修改
2. ur_mordern_driver：ur机器人通讯控制功能包，由UR提供，基本上不需要修改
3. robot_rviz_sim: 包含一个速度位置控制功能包和三个机器人描述包。
   iai_stuff：机器人关节的速度和位置控制功能包，基本不需要修改
   ur_description: ur的几何描述文件和控制器描述文件
   agv_description: agv的几何描述文件和控制器描述文件
   human_description: human的几何描述文件和控制器描述文件
   增加新的控制单元时，只需要增加对应单元的几何描述文件和控制器描述文件即可
4. mm_rviz_sim: 仿真整合功能包，整合了ur5,agv,human,table,rg2,env等几何信息。并且将ur5,agv和human的速度位置接口进行了整合。目前不同的控制单元可以放在不同的命名空间中，方便进行控制。   详细的修改规则以及控制器内部的话题传递机制，参考对应包中的readme
5. modified_tf: 辅助功能包，主要负责话题整合和修改
