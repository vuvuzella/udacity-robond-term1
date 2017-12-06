import numpy as np
class State:
    def run (self, inputs):
        assert 0, "run not Implemented"

    def next (self, inputs):
        assert 0, "next not implemented"

class StateMachine:
    def __init__(self, initialState, inputs=None):
        self.currentState = initialState
        self.currentState.run(inputs)

    def run(self, inputs):
        self.currentState = self.currentState.next(inputs)
        self.currentState.run(inputs)


#########################
#   States
##########################
class StopState(State):
    def run(self, Rover):
        print("Stopping")
        if Rover:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0

    def next(self, Rover):
        if Rover.vel > 0.2:  # arbitrary stopping threshold
            return RoverSM.stop
        else:
            if len(Rover.nav_angles) >= Rover.go_forward:
                return RoverSM.forward
            return RoverSM.rotateLeft

class ForwardState(State):
    def run(self, Rover):
        print("Forward")
        if Rover.vel < Rover.max_vel:
            Rover.throttle += Rover.throttle_set
        else:
            Rover.throttle = 0
        # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
        Rover.brake = 0

    def next(self, Rover):
        if len(Rover.nav_angles) >= Rover.stop_forward:
            return RoverSM.forward
        else:
            return RoverSM.stop

class RotateLeftState(State):
    def run(self, Rover):
        print("Rotate Left")
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)


    def next(self, Rover):
        if len(Rover.nav_angles) >= Rover.stop_forward:
            return RoverSM.forward
        else:
            return RoverSM.rotateLeft

class RotateRightState(State):
    def run(self, inputs):
        print("RotateRight")

    def next(self, inputs):
        return RoverSM.stop

class RoverSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, RoverSM.stop)

RoverSM.stop = StopState()
RoverSM.forward = ForwardState()
RoverSM.rotateLeft = RotateLeftState()
RoverSM.rotateRight = RotateRightState()