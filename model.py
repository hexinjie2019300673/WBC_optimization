import mujoco
import mujoco.viewer
import time
import socket 
import struct 

#socket 通信 客户端
sock_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 设置服务器地址和端口
addr_srv = ('127.0.0.1', 6000)
# 连接到服务器
sock_client.connect(addr_srv)
# 发送消息给服务器
sock_client.send("hello".encode()) 
# 接收服务器发送的消息并打印
recv_buf = sock_client.recv(1024).decode()
print(recv_buf)
# 关闭客户端套接字
sock_client.close()


# 加载模型
model = mujoco.MjModel.from_xml_path("./HYY_robot/scene.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 显示世界坐标系
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    
    # 获取关节数量和信息
    n_joints = model.nu
    joint_names = [model.joint(i).name for i in range(n_joints)]
    print(f"可控制的关节数量: {n_joints}")
    print(f"关节名称: {joint_names}")
    
    # 设置初始关节角度
    initial_qpos = [0.0] * n_joints  # 初始化为0，根据你的机器人调整
    for i in range(n_joints):
        data.ctrl[i] = initial_qpos[i]
    
    print("按 ESC 退出查看器")
    while viewer.is_running():
        # 示例：随时间变化关节角度
        for i in range(n_joints):        
            # 这里只是一个示例，让关节做简单的正弦运动
            # 你可以替换为你自己的控制逻辑
            data.ctrl[i] = data.ctrl[i] + 0 
        
        # 步进仿真
        mujoco.mj_step(model, data)
        
        # 同步查看器
        viewer.sync()
        
        # 添加小的延迟以避免运行过快
        time.sleep(0.001)