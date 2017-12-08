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
        self.currentState.run(inputs)
        self.currentState = self.currentState.next(self.currentState, inputs)


#########################
#   States
##########################
class StopState(State):
    def run(self, Rover):
        print("Stopping")
        if Rover:   # there is a possibility that Rover is none!
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0

    def next(self, prevState, Rover):
        if Rover.vel > 0.2 \
            or Rover.vel < -0.1: # arbitrary stopping threshold
            return RoverSM.stop
        else:
            if len(Rover.nav_angles) >= Rover.go_forward:
                return RoverSM.forward
            return RoverSM.rotateLeft

class ForwardState(State):
    def run(self, Rover):
        print("Forward")
        # print('Rover.roll: ' + str(np.tan(Rover.roll * np.pi / 180)))
        if Rover.vel < Rover.max_vel:
            # Rover.roll < 300:
            Rover.throttle += Rover.throttle_set
        else:
            Rover.throttle = 0
        if (np.mean(Rover.nav_angles) > 0.1) or \
            (np.mean(Rover.nav_angles) < -0.1):
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
        else:
            Rover.steer = 0
        Rover.brake = 0

    def next(self, prevState, Rover):
        if len(Rover.rock_angles) > Rover.rock_detect_thresh:
            return RoverSM.rockStop
        elif len(Rover.nav_angles) >= Rover.stop_forward:
            return RoverSM.forward
        else:
            return RoverSM.stop

class RotateLeftState(State):
    def run(self, Rover):
        print("Rotate Left")
        Rover.steer = -15
        Rover.throttle = 0
        Rover.brake = 0

    def next(self, prevState, Rover):
        if (len(Rover.nav_angles) >= Rover.stop_forward):
            if (np.mean(Rover.nav_angles) < 0.3) and \
                (np.mean(Rover.nav_angles) > -0.3):
               return RoverSM.forward
            return RoverSM.rotateLeft
        else:
            return RoverSM.rotateLeft

class RockStopState(State):
    def run(self, Rover):
        print("RockStop")
        if Rover:   # there is a possibility that Rover is none!
            Rover.throttle = 0
            Rover.brake += Rover.brake_set
            Rover.steer = 0

    def next(self, prevState, Rover):
        # if velocity is greater than threshold, ret RockStopState
        # if rock_angles is NaN or length is lesser than threshold, ret RockScanState
        # if rock_angles is equal or greater than threshold, ret RockRotate
        # if rock_distance is equal or less than threshold, ret RockPickState
        # else, return to non-rock stop
        if Rover.vel > 0.2 \
            or Rover.vel < -0.1: # arbitrary stopping threshold
            return RoverSM.rockStop
        else:
            if prevState is RockScanState:
                return Rover.stop
            else:
                if len(Rover.rock_angles) > Rover.rock_detect_thresh:
                    return RoverSM.rockRotate
                else:
                    return RoverSM.rockScan

class RockScanState(State):
    def run(self, Rover):
        print("RockScan")
        if len(Rover.rock_angles) <= Rover.rock_detect_thresh:
            Rover.steer = -15   # steer left by default
            Rover.throttle = 0
            Rover.brake = 0

        Rover.scanStartAngle += Rover.yaw - Rover.scanStartAngle

    def next(self, prevState, Rover):
        if isinstance(prevState, RockScanState):
            print(Rover.scanStartAngle)
        if isinstance(prevState, RockScanState) \
            and (Rover.scanStartAngle > 5):
            return RoverSM.rockScan
        else:
            Rover.scanStartAngle = 0
            if len(Rover.rock_angles) > Rover.rock_detect_thresh:
                return RoverSM.rockRotate
            else:
                return RoverSM.stop

class RockRotateState(State):
    def run(self, Rover):
        print("RockRotate")
        Rover.brake = 0
        Rover.throttle = 0
        if (np.mean(Rover.rock_angles) < 0):
            Rover.steer = -15
        else:
            Rover.steer = 15

    def next(self, prevState, Rover):
        if (len(Rover.rock_angles) >= Rover.rock_detect_thresh):
            if (np.mean(Rover.rock_angles) < 0.3) and \
                (np.mean(Rover.rock_angles) > -0.3):
                return RoverSM.rockForward
            return RoverSM.rockRotate
        return RoverSM.rockScan

class RockForwardState(State):
    def run(self, Rover):
        print("RockForward")
        if Rover.vel < Rover.max_vel:
            Rover.throttle = 0.5
        else:
            Rover.throttle = 0
        if (np.mean(Rover.rock_angles) > 0.1) or \
            (np.mean(Rover.rock_angles) < -0.1):
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
        else:
            Rover.steer = 0
        Rover.brake = 0

    def next(self, prevState, Rover):
        if (np.mean(Rover.rock_dists) < Rover.rock_dist_thresh):
            return RoverSM.rockStop
        return RoverSM.rockForward

class RockPickState(State):
    def run(self, Rover):
        print("RockPick")
        Rover.send_pickup = True
    
    def next(self, prevState, Rover):
        if len(Rover.rock_angles) > Rover.rock_detect_thresh:
            return RoverSM.rockForward
        return RoverSM.stop



class RoverSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, RoverSM.stop)

RoverSM.stop = StopState()
RoverSM.forward = ForwardState()
RoverSM.rotateLeft = RotateLeftState()

RoverSM.rockForward = RockForwardState()
RoverSM.rockScan = RockScanState()
RoverSM.rockRotate = RockRotateState()
RoverSM.rockStop = RockStopState()
RoverSM.rockPick = RockPickState()