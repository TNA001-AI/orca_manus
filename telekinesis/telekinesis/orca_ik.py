#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
import os

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 
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
        self.show_gui = self.declare_parameter('show_gui', True).get_parameter_value().bool_value
        p.connect(p.GUI if self.show_gui else p.DIRECT)
        # load right orca hand      
        package_share_dir = get_package_share_directory('telekinesis')
        # print(package_share_dir)
        path_src = os.path.join(package_share_dir, 'telekinesis')
        self.is_left = self.declare_parameter('isLeft', False).get_parameter_value().bool_value
        self.glove_to_orca_mapping_scale = 1.3
        self.urdfEndEffectorIndex = [
            8,  12,   # thumb  PIP & fingertip
            17, 19,   # index  PIP & fingertip
            24, 26,   # middle PIP & fingertip
            31, 33,   # ring   PIP & fingertip
            38, 40    # pinky  PIP & fingertip
        ]
        if self.is_left:
            urdf_path = os.path.join(path_src, URDF_PATH)
            print(f"Loading URDF from: {urdf_path}")
            # You may have to set this path for your setup on ROS2
            self.OrcaId = p.loadURDF(
                urdf_path,
                [0, 0, 0],
                p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase = True
            )
               
            self.pub_hand = self.create_publisher(JointState, PUBLISH_TOPIC_NAME, qos_profile_sensor_data)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 1)
        else:
            urdf_path = os.path.join(path_src, URDF_PATH)
            print(f"Loading URDF from: {urdf_path}")
            self.OrcaId = p.loadURDF(
                urdf_path,
                [0, 0, 0],
                p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase = True
            )
            
            self.pub_hand = self.create_publisher(JointState, PUBLISH_TOPIC_NAME, qos_profile_sensor_data)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/r_short", self.get_glove_data, 1)

        self.numJoints = p.getNumJoints(self.OrcaId)

        print("\nAll links in URDF:")
        print("Base link index: -1")
        for i in range(self.numJoints):
            joint_info = p.getJointInfo(self.OrcaId, i)
            print(f"Link {i}: {joint_info[12].decode('utf-8')}")  # joint_info[12] is the link name
        print("\nAll joints in URDF:")
        for i in range(self.numJoints):
            joint_info = p.getJointInfo(self.OrcaId, i)
            print(f"Joint {i}: {joint_info[1].decode('utf-8')}")  # joint_info[1] is the joint name
        
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()
            
    def create_target_vis(self):
        # load balls
        small_ball_radius = 0.005
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.005
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]
        
        self.ballMbt = []
        for i in range(10):  # Create 10 balls for all positions
            self.ballMbt.append(p.createMultiBody(
                baseMass=baseMass, 
                baseCollisionShapeIndex=ball_shape, 
                basePosition=basePosition
            )) # for all finger positions    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        
        # Set ball colors
        # Thumb
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 0.7])  # Red - Thumb middle
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[1, 0, 0, 1])    # Red - Thumb tip
        
        # Index
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 1, 0, 0.7])  # Green - Index middle
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[0, 1, 0, 1])    # Green - Index tip
        
        # Middle
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[0, 0, 1, 0.7])  # Blue - Middle middle
        p.changeVisualShape(self.ballMbt[5], -1, rgbaColor=[0, 0, 1, 1])    # Blue - Middle tip
        
        # Ring
        p.changeVisualShape(self.ballMbt[6], -1, rgbaColor=[1, 1, 0, 0.7])  # Yellow - Ring middle
        p.changeVisualShape(self.ballMbt[7], -1, rgbaColor=[1, 1, 0, 1])    # Yellow - Ring tip
        
        # Pinky
        p.changeVisualShape(self.ballMbt[8], -1, rgbaColor=[1, 1, 1, 0.7])  # White - Pinky middle
        p.changeVisualShape(self.ballMbt[9], -1, rgbaColor=[1, 1, 1, 1])    # White - Pinky tip

    def update_target_vis(self, hand_pos):
        """
        Update the visualization balls' positions
        
        Args:
            hand_pos: List of positions for each finger
            - hand_pos[0]: Thumb middle position
            - hand_pos[1]: Thumb tip position
            - hand_pos[2]: Index middle position
            - hand_pos[3]: Index tip position
            - hand_pos[4]: Middle middle position
            - hand_pos[5]: Middle tip position
            - hand_pos[6]: Ring middle position
            - hand_pos[7]: Ring tip position
            - hand_pos[8]: Pinky middle position
            - hand_pos[9]: Pinky tip position
        """
        # Update each ball's position
        for i in range(10):
            # Get current ball position and orientation
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[i])
            # Update ball position while maintaining orientation
            p.resetBasePositionAndOrientation(
                self.ballMbt[i], 
                hand_pos[i], 
                current_orientation
            )
        
    def get_glove_data(self, pose):
        # gets the data converts it and then computes IK and visualizes
        poses = pose.poses
        # print(f"Received {len(poses)} poses from glove")
        hand_pos = []  
        for i in range(0,10):
            # Modify coordinate mapping
            x = poses[i].position.x * self.glove_to_orca_mapping_scale
            y = - poses[i].position.y * self.glove_to_orca_mapping_scale
            z = poses[i].position.z * self.glove_to_orca_mapping_scale 
            hand_pos.append([x, y, z])
        # key = index in hand_pos   value = (dx, dy, dz)
        offsets = {
            1: (0.020, 0.030, 0.000),
            0: (0.020, 0.030, 0.000),
            3: (0.000, 0.000, 0.000),
            2: (0.000, 0.000, 0.000),
            5: (0.000, 0.000, 0.000),
            4: (0.000, 0.000, 0.000),
            7: (0.000, 0.000, 0.000),
            6: (0.000, 0.000, 0.000),
            9: (0.000, 0.000, 0.030),
            8: (0.000, 0.000, 0.030),
        }

        for idx, (dx, dy, dz) in offsets.items():
            hand_pos[idx][0] += dx
            hand_pos[idx][1] += dy
            hand_pos[idx][2] += dz
        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)
    # ---------------------------------------------------------------------
    # IK computation & command publishing
    # ---------------------------------------------------------------------
    def compute_IK(self, hand_pos):
        p.stepSimulation()     

        # ------------------------------------------------------------------
        # 1. Build the target list for the IK solver (order **must** match
        #    `self.urdf_end_effector_index`). We use both the middle‑phalanx and
        #    fingertip of each finger for greater stability.
        # ------------------------------------------------------------------
        orca_end_effector_pos = [
            hand_pos[0], hand_pos[1],  # thumb  mid‑ & fingertip
            hand_pos[2], hand_pos[3],  # index  mid‑ & fingertip
            hand_pos[4], hand_pos[5],  # middle mid‑ & fingertip
            hand_pos[6], hand_pos[7],  # ring   mid‑ & fingertip
            hand_pos[8], hand_pos[9],  # pinky  mid‑ & fingertip
        ]

        # ------------------------------------------------------------------
        # 2. Solve IK with PyBullet's damped‑least‑squares (DLS) solver.
        #    The result length equals the number of **movable** joints, i.e.
        #    16 for the ORCA hand (thumb + 4 fingers × 3 DOF each, plus
        #    1 abduction DOF per finger).
        # ------------------------------------------------------------------    
        jointPoses = p.calculateInverseKinematics2(
            self.OrcaId,
            self.urdfEndEffectorIndex,
            orca_end_effector_pos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        # ------------------------------------------------------------------
        # 3. Copy the IK solution into a full‑length joint array so we can send
        #    POSITION_CONTROL commands to PyBullet for visual feedback.
        # ------------------------------------------------------------------

        active_joint_indices = [
            i for i in range(self.numJoints)
            if p.getJointInfo(self.OrcaId, i)[2] != p.JOINT_FIXED       # type!=FIXED
        ]

        all_joint_positions = [0.0] * self.numJoints
        for idx_in_list, joint_idx in enumerate(active_joint_indices):
            all_joint_positions[joint_idx] = jointPoses[idx_in_list]

        finger_joints = active_joint_indices          # 对 ORCA HAND 刚好就是 16 个
        for joint_idx in finger_joints:
            p.setJointMotorControl2(
                bodyIndex=self.OrcaId,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=all_joint_positions[joint_idx],
                force=500,
                positionGain=0.3,
                velocityGain=1.0,
            )

        # ------------------------------------------------------------------
        # 4. Map the IK angles to the 17‑element vector expected by the real
        #    ORCA hand driver: index 0 = wrist (unused here), 1‑16 = fingers.
        # ------------------------------------------------------------------

        
        stater = JointState()
        stater.header.stamp = self.get_clock().now().to_msg()
        stater.position = [0.0] + [float(i) for i in jointPoses]
        stater.name = [
            'right_wrist',  
            'right_thumb_mcp', 'right_thumb_abd', 'right_thumb_pip', 'right_thumb_dip',
            'right_index_abd', 'right_index_mcp', 'right_index_pip',
            'right_middle_abd', 'right_middle_mcp', 'right_middle_pip',
            'right_ring_abd', 'right_ring_mcp', 'right_ring_pip',
            'right_pinky_abd', 'right_pinky_mcp', 'right_pinky_pip',
        ]
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