from struct import pack
from time import sleep
import pprint
from mouseController import mouse_move_right 
from ControllerInfoGrabber import ControllerInfoGrabber, NoControllerError, NoUpdateError

cig = ControllerInfoGrabber()

# Constants for left and right
LEFT = 0 
RIGHT = 1

# Three connection indicators:
connection_status = False 
left_controller_status = False 
right_controller_status = False

# The current packet number for left & right controllers. 
# A change in this number means an update from this controller. 
# Initializing them to 0. 
packetLeft = 0
packetRight = 0

# The frequency of info update
Hz = 250


connection_status = cig.connect()

def visualizer(d):
    for key in d:
        current_value = d[key]
        if key == "angle":
            break
        elif current_value >= 0:
            d[key] =  "+++++" * int(current_value) 
        else:
            d[key] =  "-----" * (-1 * int(current_value))

# cig.main()

pp = pprint.PrettyPrinter(indent=4)

while True:
    sleep(1.0/Hz)
    left_id, right_id = cig.connectController()
    try:
        left_success, left_pose, left_state, left_event = cig.getInfo(packetNum=packetLeft, deviceID=left_id)
        # print(left_success)
        # visualizer(left_pose)
        # print(left_pose)
        # if left_event is not None: 
        #     pp.pprint(left_event)
    except NoControllerError:
        # print("Left Controller Disconnected...")
        pass

    try:
        right_success, right_pose, right_state, right_event = cig.getInfo(packetNum=packetRight, deviceID=right_id)
        # print(right_success)
        # visualizer(right_pose)
        # print(right_pose)
        
        if right_event is not None: 
            pp.pprint(right_event)
            pp.pprint(right_state)
            # print("Button:")
            # print(dir(right_event))
            # print(right_event.eventType)
            # print(right_event.trackedDeviceIndex)

    except NoControllerError:
        # print("Right Controller Disconnected...")
        pass
