#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
import os

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
import sys
from ament_index_python.packages import get_package_share_directory
'''
This takes the glove data, and runs inverse kinematics and then publishes onto Orca Hand.

Note how the fingertip positions are matching, but the joint angles between the two hands are not.  :) 

Inspired by Dexcap https://dex-cap.github.io/ by Wang et. al. and Robotic Telekinesis by Shaw et. al.
'''
PUBLISH_TOPIC_NAME = "/joint_states"
URDF_PATH = "orca_hand/orcahand_right_wrist_fixed.urdf"

class OrcaPybulletIK(Node):
    def __init__(self):
        super().__init__('ORCA_Pybullet_IK')  
        # start pybullet
        p.connect(p.GUI)
        # load right orca hand      
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        self.is_left = self.declare_parameter('isLeft', False).get_parameter_value().bool_value
        self.glove_to_orca_mapping_scale = 1.6
        self.orcaEndEffectorIndex = [3, 4, 8, 9, 13, 14, 18, 19, 23, 24]
        if self.is_left:
            path_src = os.path.join(path_src, URDF_PATH)
            # You may have to set this path for your setup on ROS2
            self.OrcaId = p.loadURDF(
                path_src,
                [0, 0, 0],
                p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase = True
            )
               
            self.pub_hand = self.create_publisher(JointState, PUBLISH_TOPIC_NAME, 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 10)
        else:
            path_src = os.path.join(path_src, URDF_PATH)

            self.OrcaId = p.loadURDF(
                path_src,
                [0, 0, 0],
                p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase = True
            )
            
            self.pub_hand = self.create_publisher(JointState, PUBLISH_TOPIC_NAME, 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/r_short", self.get_glove_data, 10)

        self.numJoints = p.getNumJoints(self.OrcaId)
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()
            
    def create_target_vis(self):
        # load balls
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]
        
        self.ballMbt = []
        for i in range(5):
            self.ballMbt.append(p.createMultiBody(
                baseMass=baseMass, 
                baseCollisionShapeIndex=ball_shape, 
                basePosition=basePosition
            )) # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 0, 1])
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[1, 1, 1, 1])
        
    def update_target_vis(self, hand_pos):
        """
        Update the visualization balls' positions
        
        Args:
            hand_pos: List of positions for each finger tip
            - hand_pos[1]: Thumb tip position
            - hand_pos[2]: Index finger tip position
            - hand_pos[3]: Middle finger tip position
            - hand_pos[7]: Ring finger tip position
            - hand_pos[8]: Pinky finger tip position
        """
        # 定义球体索引和对应的手指位置索引
        ball_to_finger_map = {
            0: 1,  # Thumb
            1: 2,  # Index finger
            2: 3,  # Middle finger
            3: 7,  # Ring finger
            4: 8   # Pinky finger
        }
        
        # 更新每个球体的位置
        for ball_idx, pos_idx in ball_to_finger_map.items():
            # 获取当前球体的位置和方向
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[ball_idx])
            # 更新球体位置，保持原有方向
            p.resetBasePositionAndOrientation(
                self.ballMbt[ball_idx], 
                hand_pos[pos_idx], 
                current_orientation
            )
        
    def get_glove_data(self, pose):
        # gets the data converts it and then computes IK and visualizes
        poses = pose.poses
        hand_pos = []  
        for i in range(0,10):
            hand_pos.append([poses[i].position.x * self.glove_to_orca_mapping_scale * 1.15, poses[i].position.y * self.glove_to_orca_mapping_scale, -poses[i].position.z * self.glove_to_orca_mapping_scale])
        # hand_pos[2][0] = hand_pos[2][0] - 0.02  this isn't great because they won't oppose properly
        # hand_pos[3][0] = hand_pos[3][0] - 0.02    
        # hand_pos[6][0] = hand_pos[6][0] + 0.02
        # hand_pos[7][0] = hand_pos[7][0] + 0.02
        # hand_pos[2][1] = hand_pos[2][1] + 0.002
        hand_pos[4][1] = hand_pos[4][1] + 0.002
        hand_pos[6][1] = hand_pos[6][1] + 0.002
        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)
        
    def compute_IK(self, hand_pos):
        p.stepSimulation()     

        rightHandThumb_middle_pos = hand_pos[0]
        rightHandThumb_pos = hand_pos[1]

        rightHandIndex_middle_pos = hand_pos[2]
        rightHandIndex_pos = hand_pos[3]
        
        rightHandMiddle_middle_pos = hand_pos[4]
        rightHandMiddle_pos = hand_pos[5]
        
        rightHandRing_middle_pos = hand_pos[6]
        rightHandRing_pos = hand_pos[7]
        
        rightHandPinky_middle_pos = hand_pos[8]
        rightHandPinky_pos = hand_pos[9]

        
        orcaEndEffectorPos = [
            rightHandThumb_middle_pos,
            rightHandThumb_pos,
            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandRing_middle_pos,
            rightHandRing_pos,
            rightHandPinky_middle_pos,
            rightHandPinky_pos
        ]

        print("End effector indices:", self.orcaEndEffectorIndex)
        for idx in self.orcaEndEffectorIndex:
            joint_info = p.getJointInfo(self.OrcaId, idx)
            print(f"Joint {idx}: {joint_info[1]}") 
            
        jointPoses = p.calculateInverseKinematics2(
            self.OrcaId,
            self.orcaEndEffectorIndex,
            orcaEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        
        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,) + jointPoses[12:16] + (0.0,))
        combined_jointPoses = list(combined_jointPoses)

        # update the hand joints
        for i in range(20):
            p.setJointMotorControl2(
                bodyIndex=self.OrcaId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        joint_names = {
            'right_wrist': 0,
            'right_thumb_mcp': 1,
            'right_thumb_abd': 2,
            'right_thumb_pip': 3,
            'right_thumb_dip': 4,
            'right_index_abd': 5,
            'right_index_mcp': 6,
            'right_index_pip': 7,
            'right_middle_abd': 8,
            'right_middle_mcp': 9,
            'right_middle_pip': 10,
            'right_ring_abd': 11,
            'right_ring_mcp': 12,
            'right_ring_pip': 13,
            'right_pinky_abd': 14,
            'right_pinky_mcp': 15,
            'right_pinky_pip': 16
        }

        # map results to real robot
        real_robot_hand_q = np.array([float(0.0) for _ in range(17)])

        # Assign joint angles using joint names
        # Thumb joints
        real_robot_hand_q[joint_names['right_thumb_mcp']] = jointPoses[0]
        real_robot_hand_q[joint_names['right_thumb_abd']] = jointPoses[1]
        real_robot_hand_q[joint_names['right_thumb_pip']] = jointPoses[2]
        real_robot_hand_q[joint_names['right_thumb_dip']] = jointPoses[3]

        # Index finger joints
        real_robot_hand_q[joint_names['right_index_abd']] = jointPoses[4]
        real_robot_hand_q[joint_names['right_index_mcp']] = jointPoses[5]
        real_robot_hand_q[joint_names['right_index_pip']] = jointPoses[6]

        # Middle finger joints
        real_robot_hand_q[joint_names['right_middle_abd']] = jointPoses[7]
        real_robot_hand_q[joint_names['right_middle_mcp']] = jointPoses[8]
        real_robot_hand_q[joint_names['right_middle_pip']] = jointPoses[9]

        # Ring finger joints
        real_robot_hand_q[joint_names['right_ring_abd']] = jointPoses[10]
        real_robot_hand_q[joint_names['right_ring_mcp']] = jointPoses[11]
        real_robot_hand_q[joint_names['right_ring_pip']] = jointPoses[12]

        # Pinky finger joints
        real_robot_hand_q[joint_names['right_pinky_abd']] = jointPoses[13]
        real_robot_hand_q[joint_names['right_pinky_mcp']] = jointPoses[14]
        real_robot_hand_q[joint_names['right_pinky_pip']] = jointPoses[15]

        stater = JointState()
        stater.position = [float(i) for i in real_robot_hand_q]
        self.pub_hand.publish(stater)

def main(args=None):
    rclpy.init(args=args)
    orcapybulletik = OrcaPybulletIK()
    rclpy.spin(orcapybulletik)
    # Destroy the node explicitly
    orcapybulletik.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()