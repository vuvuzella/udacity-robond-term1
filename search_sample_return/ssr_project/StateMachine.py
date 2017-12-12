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
                # if len(Rover.rock_angles) > Rover.rock_detect_thresh:
                #     return RoverSM.rockRotate
                # else:
                if isinstance(prevState, RockForwardState):
                    return RoverSM.rockPick
                else:
                    return RoverSM.rockScanLeft

class RockScanLeftState(State):
    def run(self, Rover):
        print("RockScanLeft")
        if Rover.scanStart == False:
            Rover.scanStart = True
            Rover.scanOriginalAngle = Rover.yaw
            # scan left
            # if Rover.scanOriginalAngle > 270:
            #     Rover.scanDestAngle = 360 - (Rover.scanOriginalAngle + 90)
            # else:
            #     Rover.scanDestAngle = Rover.scanOriginalAngle + 90
            tempLeftDest = Rover.scanOriginalAngle + 90
            # if tempLeftDestLef > 360:
            #     Rover.scanDestAngle = 360 - (Rover.scanOriginalAngle + 90)
            # else:
            Rover.scanDestAngle = tempLeftDest

        Rover.steer = 15
        Rover.throttle = 0
        Rover.brake = 0

        print("scanOriginalAngle: " + str(Rover.scanOriginalAngle))
        print("scanDestAngle: " + str(Rover.scanDestAngle))
        print("Rover.yaw: " + str(Rover.yaw))

    def next(self, prevState, Rover):
        if Rover.scanDestAngle > 360:
            if Rover.yaw < 90:
                if Rover.yaw >= (Rover.scanDestAngle - 360):
                    Rover.rockScanLeft = True
                    Rover.scanStart = False
                    return RoverSM.rockScanCenter
                else:
                    return RoverSM.rockScanLeft
            else:
                if Rover.yaw >= Rover.scanDestAngle:
                    print("This should not print!!!!!!!!!!!!!!!!!!!!!!!!1")
                    Rover.rockScanLeft = True
                    Rover.scanStart = False
                    return RoverSM.rockScanCenter
                else:
                    return RoverSM.rockScanLeft
        else:
            if Rover.yaw >= Rover.scanDestAngle:
                Rover.rockScanLeft = True
                Rover.scanStart = False
                return RoverSM.rockScanCenter
            else:
                return RoverSM.rockScanLeft

class RockScanRightState(State):
    def run(self, Rover):
        print("RockScanRight")
        if Rover.scanStart == False:
            Rover.scanStart = True
            Rover.scanOriginalAngle = Rover.yaw
            # scan right
            # if Rover.scanOriginalAngle < 90:
            #     Rover.scanDestAngle = 360 - np.abs(Rover.scanOriginalAngle - 90)
            # else:
            #     Rover.scanDestAngle = Rover.scanOriginalAngle + 90
            tempRightDest = Rover.scanOriginalAngle - 90
            # if tempRightDest < 0:
            #     Rover.scanDestAngle = 360 - np.abs(tempRightDest)
            # else:
            Rover.scanDestAngle = tempRightDest

        Rover.steer = -15
        Rover.throttle = 0
        Rover.brake = 0

        print("scanOriginalAngle: " + str(Rover.scanOriginalAngle))
        print("scanDestAngle: " + str(Rover.scanDestAngle))
        print("Rover.yaw: " + str(Rover.yaw))

    def next(self, prevState, Rover):
        if Rover.scanDestAngle < 0:
            if Rover.yaw > 270:
                if Rover.yaw < (360 - Rover.scanDestAngle):
                    return RoverSM.rockScanRight
                else:
                    Rover.rockScanRight = True
                    Rover.scanStart = False
                    return RoverSM.rockScanCenter
            else:
                return RoverSM.rockScanRight
        else:
            if Rover.yaw < Rover.scanDestAngle:
                return RoverSM.rockScanRight
            else:
                Rover.rockScanRight = True
                Rover.scanStart = False
                return RoverSM.rockScanCenter

class RockScanCenterState(State):
    def run(self, Rover):
        print("RockScanCenter")
        if Rover.scanStart == False:
            Rover.scanStart = True
            # Rover.scanOriginalAngle = Rover.yaw
            # scan center
            Rover.scanDestAngle = Rover.scanOriginalAngle

        if Rover.scanLeft == True:
            Rover.steer = -15
        else:
            Rover.steer = 15
        Rover.throttle = 0
        Rover.brake = 0

        print("scanOriginalAngle: " + str(Rover.scanOriginalAngle))
        print("scanDestAngle: " + str(Rover.scanDestAngle))
        print("Rover.yaw: " + str(Rover.yaw))

    def next(self, prevState, Rover):
        if Rover.scanLeft == True:
            if Rover.scanDestAngle > 270:
                if Rover.yaw >= Rover.scanOriginalAngle:
                    # finished
                    Rover.scanLeft = False
                else:
                    return RoverSM.rockScanCenter
            else:
                if Rover.yaw < Rover.scanOriginalAngle:
                    return RoverSM.rockScanRight
                else:
                    return RoverSM.rockScanCenter
        else:
            if Rover.scanDestAngle < 90:
                if Rover.yaw < Rover.scanDestAngle:
                    # finished
                    Rover.scanRight = False
                else:
                    return RoverSM.rockScanCenter
            else:
                if Rover.yaw > Rover.scanOriginalAngle:
                    return RoverSM.stop
                else:
                    return RoverSM.rockScanCenter

class RockScanState(State):
    def run(self, Rover):
        print("RockScan")

        # if Rover.scanStart == False:
        #     Rover.scanStart = True
        #     Rover.scanStartAngle = Rover.yaw
        #     Rover.scanOriginalAngle = Rover.yaw
        #     Rover.scanAngleDiff = 0

        # Rover.throttle = 0
        # Rover.brake = 0

        # ssAngle = 0
        # yawAngle = 0

        # if Rover.scanLeft == False:
        #     # scan left
        #     print("Scanning Left")
        #     Rover.steer = 15    # steer left
        #     if Rover.scanAngleDiff >= (90):
        #         # scanning left is done
        #         Rover.scanLeft = True
        #         Rover.scanAngleDiff = 0
        #         Rover.scanStartAngle = Rover.yaw
        #     else:
        #         if Rover.scanOriginalAngle >= 270:
        #             if Rover.yaw >= 270:
        #                 yawAngle = 360 - Rover.yaw
        #             else:
        #                 yawAngle = Rover.yaw
        #             ssAngle = 360 - Rover.scanStartAngle
        #         else:
        #             yawAngle = Rover.yaw
        #             ssAngle = Rover.scanStartAngle

        # elif Rover.scanRight == False:
        #     # scan right
        #     print("Scanning Right")
        #     Rover.steer = -15    # steer right
        #     if Rover.scanAngleDiff >= (180):
        #         Rover.scanRight = True
        #         Rover.scanAngleDiff = 0
        #         Rover.scanStartAngle = Rover.yaw
        #     else:
        #         ssAngle = Rover.scanStartAngle
        #         if Rover.scanOriginalAngle <= 90:
        #             if Rover.yaw > 270:
        #                 yawAngle = 360 - Rover.yaw
        #             else:
        #                 yawAngle = Rover.yaw
        #         else:
        #             yawAngle = Rover.yaw
        # else:
        #     # return to original
        #     Rover.steer = 15
        #     print("Going back to original")
        #     if Rover.scanAngleDiff >= (90):
        #         Rover.scanOriginal = True
        #         Rover.scanAngleDiff = 0
        #     else:
        #         # ssAngle = Rover.scanStartAngle
        #         if Rover.yaw >= 270:
        #             yawAngle = 360 - Rover.yaw
        #         if Rover.scanOriginalAngle <= 90:
        #             # if Rover.yaw >= 270:
        #             #     yawAngle = 360 - Rover.yaw
        #             # else:
        #             #     yawAngle = Rover.yaw
        #             ssAngle = 360 - Rover.scanStartAngle 
        #         else:
        #             # yawAngle = Rover.yaw
        #             ssAngle = Rover.scanStartAngle

        # if Rover.scanOriginalAngle <= 90:
        #     if Rover.yaw >= 270:
        #         Rover.scanAngleDiff = np.abs(yawAngle + ssAngle)
        #     else:
        #         Rover.scanAngleDiff = np.abs(yawAngle - ssAngle)
        # elif Rover.scanOriginalAngle >= 270:
        #     if Rover.yaw <= 90:
        #         Rover.scanAngleDiff = np.abs(yawAngle - ssAngle)
        #     else:
        #         Rover.scanAngleDiff = np.abs(yawAngle + ssAngle)
        # else:
        #     # scanOriginalAngle is in 2nd or third quadrant
        #     # calculate as is
        #     Rover.scanAngleDiff = np.abs(yawAngle - ssAngle)

        # print("Rover.yaw: " + str(Rover.yaw))
        # print("Rover.scanOriginalAngle: " + str(Rover.scanOriginalAngle))
        # print("Rover.scanStartAngle: " + str(Rover.scanStartAngle))
        # print("ssAngle: " + str(ssAngle))
        # print("yawAngle: " + str(yawAngle))
        # print("Rover.scanAngleDiff: " + str(Rover.scanAngleDiff))


    def next(self, prevState, Rover):
        pass
        # if len(Rover.rock_angles) > Rover.rock_detect_thresh \
        #     and (np.mean(Rover.rock_angles) < 0.3) \
        #     and (np.mean(Rover.rock_angles) > -0.3):
        #     Rover.scanStart = False
        #     Rover.scanLeft = False
        #     Rover.scanRight = False
        #     Rover.scanOriginal = False
        #     return RoverSM.rockForward
        # else:
        # if Rover.scanRight == True \
        #     and Rover.scanLeft == True \
        #     and Rover.scanOriginal == True:
        #     Rover.scanStart = False
        #     Rover.scanLeft = False
        #     Rover.scanRight = False
        #     Rover.scanOriginal = False
        #     return RoverSM.stop
        # else:
        #     return RoverSM.rockScan


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
        if isinstance(prevState, RockPickState):
            if Rover.picking_up == 1:
                return RoverSM.rockPick
            else:
                if len(Rover.rock_angles) > Rover.rock_detect_thresh:
                    return RoverSM.rockForward
                else:
                    return RoverSM.stop
        return RoverSM.rockPick

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

RoverSM.rockScanLeft = RockScanLeftState()
RoverSM.rockScanRight = RockScanLeftState()
RoverSM.rockScanCenter = RockScanCenterState()