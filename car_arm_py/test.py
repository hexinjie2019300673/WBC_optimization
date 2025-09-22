import swift
import roboticstoolbox as rtb

# 启动 Swift 仿真环境
env = swift.Swift()
env.launch(realtime=True)

# 加载 Frankie 模型
frankie = rtb.models.Panda()
frankie.q = frankie.qr  # 设定初始关节角度

# 添加到仿真环境
env.add(frankie)

# 设置一个好看的相机视角（可选）
env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

# 更新一次场景
env.step()

# 保持窗口
env.hold()
