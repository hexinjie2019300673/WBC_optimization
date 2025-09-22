## 双臂 静态目标  

import mujoco
import mujoco.viewer
import numpy as np 
import time
import socket 
import struct 
from scipy.spatial.transform import Rotation as R


HOST = '127.0.0.1'
PORT = 12345
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# 加载模型
model = mujoco.MjModel.from_xml_path("../HYY_robot/scene2.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 显示世界坐标系
    # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    
    # 获取关节数量和信息
    n_joints = model.nu
    joint_names = [model.joint(i).name for i in range(n_joints)]
    print(f"可控制的关节数量: {n_joints}")
    print(f"关节名称: {joint_names}")
    
    # 设置初始关节角度
    #initial_qpos = [0.0] * n_joints  # 初始化为0，根据你的机器人调整
    initial_qpos = [ 0 ,0, 
                    0 ,0 ,0 , 
                    0, -0.6, 0, -0.44, 0, -0.62, 0,
                    0, 0, 0 ,0 ,0 ,0 ,0]
    for i in range(n_joints):
        data.ctrl[i] = initial_qpos[i]
    
    car_name = 'base_link'
    car_id = model.body(name=car_name).id
    
    carbase_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base')
    qpos_addr = model.jnt_qposadr[carbase_joint_id]
    
    target_position = np.array([0, 0, 0.267954])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])  # 单位四元数
    target_R  = np.array([
    [1.0,  0.0, 0.0],
    [0.0,  1.0, 0.0],
    [0.0,  0.0, 1.0]
                        ])
    
    print("按 ESC 退出查看器")
    
    
    while viewer.is_running():

        #发送数据 
        car_rot = np.array(data.xmat[car_id]).reshape(3,3)
        car_pos = np.array(data.xpos[car_id]).reshape(3,1)
        
        star_rot1 = np.array([  [1,0,0],
                               [0,1,0],
                               [0,0,1] ])
        star_pos1 = np.array([3,0.2,1.3])
        
        star_rot2 = np.array([ [1,0,0], 
                               [0,1,0], 
                               [0,0,1]  ])
        star_pos2 = np.array([3,-0.1,1.3])
        
        data_send = [car_rot[0][0],car_rot[0][1],car_rot[0][2],car_rot[1][0],car_rot[1][1],car_rot[1][2],car_rot[2][0],car_rot[2][1],car_rot[2][2],
                     car_pos[0][0],car_pos[1][0] ,car_pos[2][0],
                     star_rot1[0][0],star_rot1[0][1],star_rot1[0][2],star_rot1[1][0],star_rot1[1][1],star_rot1[1][2],star_rot1[2][0],star_rot1[2][1],star_rot1[2][2],
                     star_pos1[0],star_pos1[1] ,star_pos1[2],
                     star_rot2[0][0],star_rot2[0][1],star_rot2[0][2],star_rot2[1][0],star_rot2[1][1],star_rot2[1][2],star_rot2[2][0],star_rot2[2][1],star_rot2[2][2],
                     star_pos2[0],star_pos2[1] ,star_pos2[2],
                     data.ctrl[2],data.ctrl[3],data.ctrl[4],
                     data.ctrl[5],data.ctrl[6],data.ctrl[7],data.ctrl[8],data.ctrl[9],data.ctrl[10],data.ctrl[11],
                     data.ctrl[12],data.ctrl[13],data.ctrl[14],data.ctrl[15],data.ctrl[16],data.ctrl[17],data.ctrl[18]]

        
        #print("data_send",data_send)
        s.sendall(struct.pack('53d',*data_send)) 
        
        
        #receive data 
        data_recv = s.recv(19 * 8)        
        recv_data = struct.unpack('19d',data_recv)


                 
        for i in range(2,n_joints):        
            data.ctrl[i] = data.ctrl[i] + recv_data[i] * 0.01
        
        # print("data_recv0",recv_data[0])
        dtheta = recv_data[0] * 0.01 
        R_rotation = np.array([
                [np.cos(dtheta),-np.sin(dtheta), 0 ],
                [np.sin(dtheta),np.cos(dtheta),0],
                [0, 0, 1]
        ])
        
        # print("target_R",target_R)
        target_R = target_R @ R_rotation 
        v_now = np.array([recv_data[1], 0 , 0])
        v_world = target_R @ v_now 
        target_position = target_position + v_world * 0.01
        
        # 转换为四元数
        r = R.from_matrix(target_R)
        quat = r.as_quat()  # 注意顺序为 [x, y, z, w]
        # MuJoCo 需要四元数格式为 [w, x, y, z]
        target_quat = np.array([quat[3], quat[0], quat[1], quat[2]])

        # print("target_quat",target_quat)
        data.qpos[qpos_addr:qpos_addr+7] = np.concatenate([target_position, target_quat])
        data.qvel[qpos_addr:qpos_addr+6] = 0  # 重置速度

        
        # 步进仿真
        mujoco.mj_step(model, data)
        
        # 同步查看器
        viewer.sync()
        
        # 添加小的延迟以避免运行过快
        time.sleep(0.01)