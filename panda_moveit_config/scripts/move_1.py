#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from control_msgs.msg import GripperCommand


class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('move_1')

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('panda_arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('hand')

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'panda_link0'
        arm.set_pose_reference_frame(reference_frame)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1.0)
        arm.set_max_velocity_scaling_factor(1.0)
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.0001)
        arm.set_goal_orientation_tolerance(0.005)
        gripper.set_goal_joint_tolerance(0.001)

        # 设置每次运动规划的时间限制：1s
        arm.set_planning_time(1)
        gripper.set_planning_time(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('ready')
        arm.go()
        rospy.sleep(0.5)

        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.2  # -0.2 0 0.2
        target_pose.pose.position.z = 0.41
        target_pose.pose.orientation.x = 1.0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()

        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(0.5)

        # 控制机械臂终端反向旋转90度  0,1,2,3,4,5 代表xyzrpy
        arm.shift_pose_target(2, -0.21, end_effector_link)
        arm.go()
        rospy.sleep(0.5)

        # 设置夹爪的目标位置，并控制夹爪运动
        # gripper.set_joint_value_target([0.0248, 0.0248])
        # gripper.go()
        # rospy.sleep(1)


"""


        # 设置机械臂的目标位置，使用七轴的位置数据进行描述（单位：弧度）
        joint_positions = [-1.5708,0.289515,-0.000185,-1.947392,0,0.6661,0]
        result = arm.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))
        arm.go()
        joint = arm.get_current_joint_values()
        print("============ Final joint: \n" + str(joint))
        pose = arm.get_current_pose('panda_hand')
        print("============ Final pose: \n" + str(pose))
        rospy.sleep(0.5)

        # 设置夹爪的目标位置，并控制夹爪运动
        gripper.set_joint_value_target([0.04,0.04])
        gripper.go()
        rospy.sleep(0.5)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('ready')
        arm.go()
        rospy.sleep(0.5)

        # 关闭并退出moveit
"""

if __name__ == "__main__":
    MoveItIkDemo()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
