# open_controller.py

##################################################################################
# This will be our control class that over the course of the next lessons will 
# become more advance!
##################################################################################

class Open_Controller:

    def __init__(self, start_time=0):
        self.start_time = start_time
        self.u = 0  # control effort
        self.last_timestamp_ = 0
        self.set_point_ = 0
        self.effort_applied = []    # all applied control efforts

    def setTarget(self, target):
        self.set_point_ = target
    
    def setControlEffort(self, control_effort):
        self.u = float(control_effort)

    def getControlEffort(self, time):
        self.last_timestamp_ = time
        self.effort_applied.append(self.u)
        return self.u