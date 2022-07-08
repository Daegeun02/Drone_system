from ast import Not
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class MotionPlanner:
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

        self.on_going_mission = None

        ## massage to control 
        self.motion_pub = rospy.Publisher("/to_control", Float32MultiArray, queue_size=1)

    def run(self):
        self.on_going_mission = self.mission_list[self.datahub.mission]
        self.on_going_mission.run()



class Motion:
    def __init__(self, datahub):
        self.datahub = datahub

    def run(self):
        return NotImplementedError()



class Disarm(Motion):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        pass



class Arm(Motion):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        pass



class Hold(Motion):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        pass



class Eject(Motion):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        pass



class EmergencyStop(Motion):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        pass



class WayPointGuidance:
    def __init__(self, datahub):
        self.datahub = datahub

    def run(self):
        return NotImplementedError()



class Takeoff(WayPointGuidance):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        # generate waypoint [0,0,15]
        # check obstacle on the way
        pass



class Trajectory(WayPointGuidance):
    def __init__(self, datahub, JPS):
        super().__init__(datahub, JPS)

        self.JPS = JPS()

    def run(self):
        
        # call JPS algorithm
        if self.datahub.obstacle_is_on_way:
            self.JPS.run()

        else:
            pass



class Land(WayPointGuidance):
    def __init__(self, datahub):
        super().__init__(datahub)

    def run(self):
        pass



class Prepare:
    def __init__(self, datahub):
        self.datahub = datahub

    def run(self):

        if not self.datahub.marker_detected:
            self.search()

        else:
            self.tracking()
                   
    def search(self):
        ## motion for detect marker
        return NotImplementedError()

    def tracking(self):
        ## after detect marker 
        return NotImplementedError()



class PrepareEject(Prepare):
    def __init__(self):
        super().__init__()



class PrepareLand(Prepare):
    def __init__(self):
        super().__init__()
