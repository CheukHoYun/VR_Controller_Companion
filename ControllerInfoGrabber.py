import ctypes
import time
import pprint
import openvr
import math
import matplotlib.pyplot as plt 

class NoControllerError(Exception):
        pass 

class NoUpdateError(Exception):
    pass

PRESS = 200 
UNPRESS = 201
TOUCH = 202
UNTOUCH = 203

class ControllerInfoGrabber:
    def __init__(self) -> None:
        self.vrsystem = None
        self.vrinput = None

    def get_controller_ids(self):
        left = None
        right = None
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vrsystem.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller:
                role = self.vrsystem.getControllerRoleForTrackedDeviceIndex(i)
                if role == openvr.TrackedControllerRole_RightHand:
                    right = i
                if role == openvr.TrackedControllerRole_LeftHand:
                    left = i
        return left, right
        

    def from_pose_to_dict(self, pose):
        p = {}

        # Get the affine matrix for transforming controller position to space position
        # Decompose it. We need the rotation around the vertical axis. 
        r = pose.mDeviceToAbsoluteTracking
        if (r[0][2]) < 1 :
                        if r[0][2] > -1:
                            thetaZ = math.atan2(-1 * (r[0][1]), r[0][0])
                        else:
                            thetaZ = 0
        cos_to_forward = math.cos(thetaZ)
        
        # By dividing the value with the cosine value vs forward, 
        # The move and tilt become local to the controller. 
        p['move_up_velocity'] = pose.vVelocity[1]
        p['move_right_velocity'] = pose.vVelocity[0] / cos_to_forward
        
        p['tilt_up_velocity'] =  pose.vAngularVelocity[0] / cos_to_forward
        p['tilt_left_velocity'] = pose.vAngularVelocity[1]
        
        p['angle'] = round(math.cos(thetaZ), 2)
        return p

    def from_event_to_dict(self, event):
        e = {} 

        # e['data'] = dir(event.data)
        e['button'] = event.data.controller.button
        e['eventAgeSeconds'] = event.eventAgeSeconds
        e['eventType'] = self.vrsystem.getEventTypeNameFromEnum(event.eventType)
        e['trackedDeviceIndex'] = event.trackedDeviceIndex
        return e

    def from_state_to_dict(self, state):
        d = {}
        d['unPacketNum'] = state.unPacketNum
        # on trigger .y is always 0.0 says the docs
        d['trigger'] = state.rAxis[1].x
        # 0.0 on trigger is fully released
        # -1.0 to 1.0 on joystick and trackpads
        d['trackpad_x'] = state.rAxis[0].x
        d['trackpad_y'] = state.rAxis[0].y
        # These are published and always 0.0
        # for i in range(2, 5):
        #     d['unknowns_' + str(i) + '_x'] = state.rAxis[i].x
        #     d['unknowns_' + str(i) + '_y'] = state.rAxis[i].y
        d['ulButtonPressed'] = state.ulButtonPressed
        d['ulButtonTouched'] = state.ulButtonTouched
        # To make easier to understand what is going on
        # Second bit marks menu button
        d['menu_button'] = bool(state.ulButtonPressed >> 1 & 1)
        # 32 bit marks trackpad
        d['trackpad_pressed'] = bool(state.ulButtonPressed >> 32 & 1)
        d['trackpad_touched'] = bool(state.ulButtonTouched >> 32 & 1)
        # d['temp'] = bin(state.ulButtonPressed)
        # third bit marks grip button
        d['grip_button'] = bool(state.ulButtonPressed >> 2 & 1)
        # System button can't be read, if you press it
        # the controllers stop reporting
        return d

    def connect(self):
        print("Initializing OpenVR")
        try:
            openvr.init(openvr.VRApplication_Scene)
            self.vrsystem = openvr.VRSystem()
            self.vrinput = openvr.VRInput()
            return True
        except openvr.OpenVRError as e:
            print("Error when initializing OpenVR.")
            return False
    
    def connectController(self):
        # print("Waiting for Controllers...")
        left_id, right_id = self.get_controller_ids()
        return left_id, right_id
    

    def getInfo(self, packetNum, deviceID):
        if deviceID is None:
            raise NoControllerError

        result, state, pose = self.vrsystem.getControllerStateWithPose(0, deviceID)

        # Lost connection to the controller:
        if result != 1:
            raise NoControllerError

        d = self.from_state_to_dict(state)
        
        event = openvr.VREvent_t()
        hasEvent = self.vrsystem.pollNextEvent(event)

        if hasEvent:
            e = self.from_event_to_dict(event)
        else:
            e = None

        
        
        p = self.from_pose_to_dict(pose)
        return (result, p, d, e)



        try:
            while True:
                time.sleep(1.0 / Hz)
                # vrChaperone.resetZeroPose(0)
                result, pControllerState, pose = vrsystem.getControllerStateWithPose(2, left_id)
                d = self.from_controller_state_to_dict(pControllerState)
                p = self.from_pose_to_dict(pose)
                if show_only_new_events and last_unPacketNum_left != d['unPacketNum']:
                    last_unPacketNum_left = d['unPacketNum']
                    print("Left controller:")
                    pp.pprint(d)
                    pp.pprint(p)
                    
                    

                result, pControllerState, pose = vrsystem.getControllerStateWithPose(0, right_id)
                d = self.from_controller_state_to_dict(pControllerState)
                p = self.from_pose_to_dict(pose)
                if show_only_new_events and last_unPacketNum_right != d['unPacketNum']:
                    last_unPacketNum_right = d['unPacketNum']
                    print("Right controller:")
                    pp.pprint(d)
                    pp.pprint(p)
                    r = p[3]
                    s = "++++++++++"
                    s2 = "----------"
                    if p[0] > 0:
                        print(s * int(p[0]))
                    else:
                        print(s2 *  int(p[0] * -1))
                    if p[1] > 0:
                        print(s * int(p[1]))
                    else:
                        print(s2 *  int(p[1] * -1))
                    if p[2] > 0:
                        print(s * int(p[2]))
                    else:
                        print(s2 * int(p[2] * -1))

                    if (r[0][2]) < 1 :
                        if r[0][2] > -1:
                            thetaZ = math.atan2(-1 * (r[0][1]), r[0][0])
                        else:
                            thetaZ = 0
                            
                    print(round(math.cos(thetaZ), 2))

        except KeyboardInterrupt:
            print("Control+C pressed, shutting down...")
            openvr.shutdown()

    def main(self):
        max_init_retries = 4
        retries = 0
        print("===========================")
        print("Initializing OpenVR...")
        while retries < max_init_retries:
            try:
                openvr.init(openvr.VRApplication_Scene)
                break
            except openvr.OpenVRError as e:
                print("Error when initializing OpenVR (try {} / {})".format(
                    retries + 1, max_init_retries))
                print(e)
                retries += 1
                time.sleep(2.0)
        else:
            print("Could not initialize OpenVR, aborting.")
            print("Make sure the system is correctly plugged, you can also try")
            print("to do:")
            print("killall -9 vrcompositor vrmonitor vrdashboard")
            print("Before running this program again.")
            exit(0)

        print("Success!")
        print("===========================")
        vrsystem = openvr.VRSystem()
        vrChaperone = openvr.VRChaperone()
        left_id, right_id = None, None
        print("===========================")
        print("Waiting for controllers...")
        try:
            while left_id is None or right_id is None:
                left_id, right_id = self.get_controller_ids(vrsystem)
                if left_id and right_id:
                    break
                print("Waiting for controllers...")
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Control+C pressed, shutting down...")
            openvr.shutdown()

        print("Left controller ID: " + str(left_id))
        print("Right controller ID: " + str(right_id))
        print("===========================")

        pp = pprint.PrettyPrinter(indent=4)

        reading_rate_hz = 250
        show_only_new_events = True
        last_unPacketNum_left = 0
        last_unPacketNum_right = 0

        print("===========================")
        print("Printing controller events!")


        try:
            while True:
                time.sleep(1.0 / reading_rate_hz)
                # vrChaperone.resetZeroPose(0)
                result, pControllerState, pose = vrsystem.getControllerStateWithPose(2, left_id)
                d = self.from_controller_state_to_dict(pControllerState)
                p = self.from_pose_to_dict(pose)
                if show_only_new_events and last_unPacketNum_left != d['unPacketNum']:
                    last_unPacketNum_left = d['unPacketNum']
                    print("Left controller:")
                    pp.pprint(d)
                    pp.pprint(p)
                    
                    

                result, pControllerState, pose = vrsystem.getControllerStateWithPose(0, right_id)
                d = self.from_controller_state_to_dict(pControllerState)
                p = self.from_pose_to_dict(pose)
                if show_only_new_events and last_unPacketNum_right != d['unPacketNum']:
                    last_unPacketNum_right = d['unPacketNum']
                    print("Right controller:")
                    pp.pprint(d)
                    pp.pprint(p)
                    r = p[3]
                    s = "++++++++++"
                    s2 = "----------"
                    if p[0] > 0:
                        print(s * int(p[0]))
                    else:
                        print(s2 *  int(p[0] * -1))
                    if p[1] > 0:
                        print(s * int(p[1]))
                    else:
                        print(s2 *  int(p[1] * -1))
                    if p[2] > 0:
                        print(s * int(p[2]))
                    else:
                        print(s2 * int(p[2] * -1))

                    if (r[0][2]) < 1 :
                        if r[0][2] > -1:
                            thetaZ = math.atan2(-1 * (r[0][1]), r[0][0])
                        else:
                            thetaZ = 0
                            
                    print(round(math.cos(thetaZ), 2))

        except KeyboardInterrupt:
            print("Control+C pressed, shutting down...")
            openvr.shutdown()