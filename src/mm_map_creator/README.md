参数修改注意事项：
1. mm_map_creator/launch/start.launch以及mm_rviz_sim/launch/ur5_agv_start.launch中的sim决定了仿真和实际机器人的程序
2. mm_sampling::isPoseinRoiy以及mm_base_placement::isPoseinRoi中的两个判断需要根据具体情况来确定
3. 