import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import zmq
'''
This reads from websockets from Manus SDK and republishes to each glove topic

The joint level data is what Manus estimates your skeleton as in the order of thumb to pinky and MCP side, MCP forward, PIP, DIP.

The full skeleton is the xyz quaternion of every single joint in the hand estimated by Manus.  The short skeleton is just the minimum data required for IK, Tip and DIP locations.

Note, currently if you are using Windows, there is a different joint ordering than with Linux.  In Windows it is thumb to pinky in finger order, but on Linux it jumps around. Please see "short_idx" for details.  
'''

IP_ADDRESS = "tcp://localhost:8000"
LEFT_GLOVE_SN =  "45a7fc8f"
RIGHT_GLOVE_SN = "8569617b"

class GloveReader(Node):
    def __init__(self):
        super().__init__('glove_reader')
        #Connect to Server
        context = zmq.Context()
        self.socket = context.socket(zmq.PULL)  
        self.socket.setsockopt(zmq.CONFLATE, True)     
        self.socket.connect(IP_ADDRESS)       
        self.pub_left = self.create_publisher(JointState, "/glove/l_joints", 10)
        self.pub_right = self.create_publisher(JointState, "/glove/r_joints", 10)

        self.pub_skeleton_right_full = self.create_publisher(PoseArray, '/glove/r_full', 1)
        self.pub_skeleton_left_full = self.create_publisher(PoseArray, '/glove/l_full', 1)

        self.pub_skeleton_right_short = self.create_publisher(PoseArray, '/glove/r_short', 1)
        self.pub_skeleton_left_short = self.create_publisher(PoseArray, '/glove/l_short', 1)
        #replace with your gloves (all lowercase letters)
        self.left_glove_sn = LEFT_GLOVE_SN
        self.right_glove_sn = RIGHT_GLOVE_SN


    #If you set a flag in the C++ code, you can send all the data that comes from the raw skeleton of the glove.  This data is from thumb to pinky, across all joints from palm to fingertip.   This can slow things down though
    def parse_full_skeleton_and_send(self, data):
        skeleton_list = []
        for i in range(0,25):
            position = Point(x=float(data[1 + i*7]), y=float(data[2 + i*7]), z=float(data[3 + i*7]))  #the first ID is right or left glove don't forget
            orientation = Quaternion(x=float(data[4 + i*7]), y=float(data[5 + i*7]), z=float(data[6 + i*7]), w=float(data[7 + i*7]))
            pose = Pose(position=position, orientation=orientation)
            skeleton_list.append(pose)
        output_array_msg = PoseArray()
        output_array_msg.poses = skeleton_list
        if data[0] == self.left_glove_sn:
            self.pub_skeleton_left_full.publish(output_array_msg) 
        elif data[0] == self.right_glove_sn:
            self.pub_skeleton_right_full.publish(output_array_msg)
        else:
            print("Glove serial number incorrect!")
            print(data[0])
    #This the dexcap style data, you only get the fingertip and the previous joint xyz as the data and then you can send that.  It goes from thumb_middle, thumb_tip, index_middle, index_tip etc.etc.
    def parse_short_skeleton_and_send(self, data):
        output_array_msg = PoseArray()
        #short_idx = [3, 4, 8, 9, 13, 14, 18, 19, 23, 24] 
        ##Right now the integrated mode is in a different ordering, pinky, thumb, index, ring, middle
        ##Will be fixed to match the SDK in a future release
        short_idx = [23, 24, 4, 5, 9, 10, 19, 20, 14, 15] 
        for i in short_idx:
            position = Point(x=float(data[1 + i*7]), y=float(data[2 + i*7]), z=float(data[3 + i*7]))  #the first ID is right or left glove don't forget
            orientation = Quaternion(x=float(0), y=float(0), z=float(0), w=float(0))
            pose = Pose(position=position, orientation=orientation)
            output_array_msg.poses.append(pose)
        if data[0] == self.left_glove_sn:
            self.pub_skeleton_left_short.publish(output_array_msg)
        elif data[0] == self.right_glove_sn:
            self.pub_skeleton_right_short.publish(output_array_msg)
        else:
            print("Glove serial number incorrect!")  
            print(data[0])      


def main(args=None):
    rclpy.init(args=args)
    glove_reader = GloveReader()
    while rclpy.ok():
        rclpy.spin_once(glove_reader, timeout_sec=0)  
        message = glove_reader.socket.recv()
        #receive the message from the socket
        message = message.decode('utf-8')
        data = message.split(",")  
        if data is not None:
            try:
                #If joint level data
                if len(data) == 40:
                    stater_msg = JointState()
                    stater_msg.position = list(map(float,data[0:20]))
                    glove_reader.pub_left.publish(stater_msg)
                    stater_msg.position = list(map(float,data[20:40]))
                    glove_reader.pub_right.publish(stater_msg)
                #If full skeleton data two hands
                elif len(data) == 352:
                    glove_reader.parse_full_skeleton_and_send(data[0:176])
                    glove_reader.parse_full_skeleton_and_send(data[176:352])
                    glove_reader.parse_short_skeleton_and_send(data[0:176])
                    glove_reader.parse_short_skeleton_and_send(data[176:352])
                #If full skeleton data one hand
                elif len(data) == 176:
                    glove_reader.parse_full_skeleton_and_send(data[0:176])
                    glove_reader.parse_short_skeleton_and_send(data[0:176])
            except KeyboardInterrupt as e:
                return
            except Exception as e:
                print(e)
                pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    glove_reader.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
