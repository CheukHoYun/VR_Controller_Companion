from ctypes import *
from ctypes import wintypes as w


import win32api, win32con

def mouse_move_right():
    win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, 100, 0, 0, 0)