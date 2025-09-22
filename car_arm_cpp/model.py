import mujoco
import mujoco.viewer
import time
import socket 
import struct 
import numpy as np 

# 加载模型
#model = mujoco.MjModel.from_xml_path("../HYY_robot/scene2.xml")

#加载模型 并显示 碰撞凸包
model = mujoco.MjModel.from_xml_path("../HYY_robot/scene_collision_vision.xml")

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 显示世界坐标系
    #viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    
    # 获取关节数量和信息
    n_joints = model.nu
    joint_names = [model.joint(i).name for i in range(n_joints)]
    print(f"可控制的关节数量: {n_joints}")
    print(f"关节名称: {joint_names}")
    
    # 设置初始关节角度
    initial_qpos = [0.0] * n_joints  # 初始化为0，根据你的机器人调整
    for i in range(n_joints):
        data.ctrl[i] = initial_qpos[i]
    #car_name = 'link-W1'
    car_name = 'L-link7'
    car_id = model.body(name=car_name).id

    print("按 ESC 退出查看器")
    while viewer.is_running():
        # 示例：随时间变化关节角度
        #获取 19 个数据 

        car_rot = np.array(data.xmat[car_id]).reshape(3,3)
        car_pos = np.array(data.xpos[car_id]).reshape(3,1)
        
        print("car_rot",car_rot) 
        print("car_pos",car_pos) 
        
        #拿到 关节角度 ， 两个坐标的 齐次矩阵
        #发送数据 
        
        joint_torques = data.qfrc_actuator.copy()
        print("关节力矩:", joint_torques)
        # 步进仿真
        mujoco.mj_step(model, data)
        
        # 同步查看器
        viewer.sync()
        
        # 添加小的延迟以避免运行过快
        time.sleep(0.01)