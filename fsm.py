from ast import Not
import rospy
import numpy as np
from std_msgs.msg import String
import time

class FSM:
    def __init__(self, datahub):
        self.datahub = datahub

        # 1. Disarm
        # 2. Arm 
        # 3. Takeoff
        # 4. Hold
        # 5. Trajectory
        # 6. PrepareEject
        # 7. Eject
        # 8. PrepareLand
        # 9. Land
        # 10. EmergencyStop

        # State
        self.mission_list = {}
        self.mission_list["Disarm"]        = Disarm(self.datahub)
        self.mission_list["Arm"]           = Arm(self.datahub)
        self.mission_list["Takeoff"]       = Takeoff(self.datahub)
        self.mission_list["Hold"]          = Hold(self.datahub)
        self.mission_list["Trajectory"]    = Trajectory(self.datahub)
        self.mission_list["PrepareEject"]  = PrepareEject(self.datahub)
        self.mission_list["Eject"]         = Eject(self.datahub)
        self.mission_list["PrepareLand"]   = PrepareLand(self.datahub)
        self.mission_list["Land"]          = Land(self.datahub)
        self.mission_list["EmergencyStop"] = EmergencyStop(self.datahub)

        # Current state
        self.on_going_mission = None

        # Mission input
        rospy.Subscriber("/mission_input", String, self.mission_callback)

    def mission_callback(self, msg):
        self.datahub.mission_input = msg.data

    def transition(self):
        self.on_going_mission = self.mission_list[self.datahub.mission]
        self.on_going_mission.transition()



# in datahub(shared), there is all sensor data.
class State:
    def __init__(self, datahub):
        self.datahub = datahub
        self.next_state = None

    # Change next state here
    def transition(self):
        return NotImplementedError()



class Disarm(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if self.datahub.mission_input == "Arm":
            self.next_state = "Arm"
            self.datahub.mission = self.next_state

        else:
            pass



class Arm(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if self.datahub.mission_input == "Disarm":
            self.next_state = "Disarm"
            self.datahub.mission = self.next_state

        elif self.datahub.mission_input == "Takeoff":
            self.next_state = "Takeoff"
            self.datahub.mission = self.next_state

        else:
            pass
    


class Takeoff(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if self.datahub.mission_input == "Land":
            self.next_state = "Land"
            self.datahub.mission = self.next_state

        else:
            pass

        if self.datahub.position[2] > 15:
            self.next_state = "Hold"
            self.datahub.mission = self.next_state

        else:
            pass



class Hold(State):
    def __init__(self, datahub):
        super().__init__(datahub)

        self.token = 0

    def transition(self):
        
        if self.datahub.mission_input == "Trajectory":
            self.next_state = "Trajectory"
            self.datahub.mission = self.next_state

        else:
            pass
        


class Trajectory(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if np.linalg.norm(self.datahub.position - self.datahub.waypoint[-1,:]) < 0.1:
            self.next_state = "PrepareEject"
            self.datahub.mission = self.next_state

        else:
            pass

        # if there's no obstacle's in our way
        if np.sum(self.datahub.voxelMap[:,:,-2:2]) == 0:
            self.obstacle_is_on_way = False
        
        # if thers's obstacle's in our way
        else:
            self.obstacle_is_on_way = True
        


class PrepareEject(State):
    def __init__(self, datahub):
        super().__init__(datahub)

        self.token = 0
        self.last_position = None
        self.live_position = None
        self.last_orientation = None
        self.live_orientation = None

    def transition(self):

        self.live_position = self.datahub.position
        self.live_orientation = self.datahub.orientation

        if self.datahub.target_marker_detected:
            if np.linalg.norm(self.live_position - self.datahub.waypoint[-1,:]) < 0.2:
                if np.linalg.norm(self.last_position - self.live_position) < 0.2:
                    if np.linalg.norm(self.last_orientation - self.live_orientation) < np.rad2deg(0.1):
                        if np.linalg.norm(self.datahub.velocity) < 0.1:
                            self.token += 1

                        else:
                            self.token = 0

                    else:
                        self.token = 0

                else:
                    self.token = 0

            else:
                self.token = 0
        
        else:
            self.token = 0

        self.last_position = self.live_position
        self.last_orientation = self.live_orientation

        if self.token < 20:
            pass

        else:
            self.token = 0
            self.next_state = "Eject"
            self.datahub.mission = self.next_state



class Eject(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):

        if self.datahub.ejected:
            self.datahub.ejected = False
            self.next_state = "Hold"
            self.datahub.mission = self.next_state

        else:
            pass



class PrepareLand(State):
    def __init__(self, datahub):
        super().__init__(datahub)

        self.token = 0

    def transition(self):
        
        ## search marker
        if self.datahub.marker_detected:
            ## track to marker's position
            if np.linalg.norm(self.datahub.position - self.datahub.marker_position):
                self.datahub.marker_detected = False
                self.next_state = "Land"
                self.datahub.mission = self.next_state

        else:
            pass
        


class Land(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):
        

        pass



class EmergencyStop(State):
    def __init__(self, datahub):
        super().__init__(datahub)

    def transition(self):
        pass
