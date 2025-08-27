import time
from src.teleop import Teleop
from piper_sdk import *
from scipy.spatial.transform import Rotation as R


def main():
    # 设置机械臂的初始末端位姿 [x,y,z,roll,pitch,yaw] xyz(m) rpy(deg)
    # Setting the initial end-effector pose of the robotic arm [x,y,z,roll,pitch,yaw] xyz(m) rpy(deg)
    init_end_pose = [0.057, 0.0, 0.25, 0.0, 89.0, 0.0]

    # 初始化遥控模块
    # Initialize the teleoperation module
    teleop = Teleop(init_end_pose)

    # 初始化并连接机械臂
    # Initialize and connect to the robotic arm
    robot = C_PiperInterface_V2()
    robot.ConnectPort()

    # 使能机械臂和夹爪
    # Enable the robotic arm and gripper
    while not robot.EnablePiper():
        time.sleep(0.01)
    robot.GripperCtrl(0, 1000, 0x01, 0)

    # 设置控制模式，0x00：位置速度控制模式；0xAD：MIT控制模式(快速响应)
    # Set control mode, 0x00: position and velocity control mode; 0xAD: MIT control mode (fast response)
    mode = 0xAD
    
    # 设置机械臂运动速度，速度范围0-100
    # Set the robotic arm movement speed, speed range 0-100
    spd = 100

    # 发送运动控制指令
    # Send motion control commands
    robot.MotionCtrl_2(0x01, 0x00, spd, mode)
    time.sleep(0.1)
    robot.MotionCtrl_2(0x01, 0x00, spd, mode)

    # 移动到初始位置
    # Move to the initial position
    ctl = [
            round(init_end_pose[0] * 1e6),
            round(init_end_pose[1] * 1e6),
            round(init_end_pose[2] * 1e6),
            round(init_end_pose[3] * 1e3),
            round(init_end_pose[4] * 1e3),
            round(init_end_pose[5] * 1e3)
        ]
    robot.EndPoseCtrl(*ctl)

    try:
        while True:
            # 控制循环延时，避免占用过多CPU资源
            # Control loop delay to avoid excessive CPU usage
            time.sleep(0.005)

            # 处理手机输入并获取目标位姿
            # Process phone input and get target pose
            pos, quat, gripper = teleop.send_to_robot()

            # 将四元数转换为欧拉角，rpy顺序，单位为度
            # Convert quaternion to Euler angles, rpy order, in degrees
            euler = R.from_quat(quat, scalar_first=True).as_euler('xyz', degrees=True)            

            # 将结果发送到机械臂
            # Send the results to the robotic arm
            # 位置单位：微米；角度单位：毫度
            # Position unit: micrometers; angle unit: millidegrees
            ctl = [
                    round(pos[0] * 1e6),
                    round(pos[1] * 1e6),
                    round(pos[2] * 1e6),
                    round(euler[0] * 1e3),
                    round(euler[1] * 1e3),
                    round(euler[2] * 1e3)
                ]
            robot.EndPoseCtrl(*ctl)

            # 控制夹爪
            # Control the gripper
            # 夹爪控制值范围0-100000
            # Gripper control value range 0-100000
            # 0：完全闭合，100000：完全张开
            # 0: fully closed, 100000: fully open
            gripper_value = 100000 if gripper else 0
            robot.GripperCtrl(gripper_value, 1000, 0x01, 0)

            # 打印当前末端位姿和夹爪状态
            # Print the current end-effector pose and gripper status
            print(f"pos: {pos}, rpy: {euler}, gripper: {gripper_value}")

    except KeyboardInterrupt:
        print("Control loop interrupted")
        teleop.server.stop(0)


if __name__ == "__main__":
    main()
