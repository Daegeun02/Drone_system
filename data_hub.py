import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class DataHub:
    def __init__(self):
        # status
        self.position = -1
        self.velocity = -1
        self.orientation = -1

        # waypoint
        self.wayPoint_to_destination = -1
        self.wayPoint_to_home        = -1
        self.marker_position         = -1

        # voxel map (20,20,20)....
        self.voxelMap = -1
        
        #  1. Disarm
        #  2. Arm 
        #  3. Takeoff
        #  4. Hold
        #  5. Trajectory
        #  6. PrepareEject
        #  7. Eject
        #  8. PrepareLand
        #  9. Land
        # 10. EmergencyStop

        self.mission = "Disarm"
        self.mission_input = -1

        # pizza delivery
        self.ejected = False

        # obstacle detect
        self.obstacle_is_on_way = False
        
        # marker detect
        self.marker_detected = False

        # initialize node
        rospy.init_node("data_hub")

        # run Subscribers
        rospy.Subscriber("/position_velocity", Float32MultiArray, self.positionVelocityCallback)
        rospy.Subscriber("/orientation", Float32MultiArray, self.orientationCallback)
        rospy.Subscriber("/voxelMap", Float32MultiArray, self.voxelMapCallback)

    def positionVelocityCallback(self, msg):

        self.position = msg.data[:3]
        self.velocity = msg.data[3:]


    def orientationCallback(self, msg):

        self.orientation = msg.data


    def voxelMapCallback(self, msg):

        self.voxelMap = msg.data