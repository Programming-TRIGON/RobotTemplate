# Copyright (c) 2023 FRC 5990
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import ntcore
import win32api
import win32con
import time

team = 5990

def main():
    ntcoreinst = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(team))
    ntcoreinst.startClient4("KeyboardToNT")
    ntcoreinst.setServer("127.0.0.1")
    ntcoreinst.startDSClient()



    # Wait for connection
    print("Waiting for connection to NetworkTables server...")
    while not ntcoreinst.isConnected():
        time.sleep(0.1)

    print("Connected!")

    # Get the table
    table = ntcoreinst.getTable("SmartDashboard/keyboard")

    keys = {
        win32con.VK_ESCAPE: "esc",
        win32con.VK_F1: "f1",
        win32con.VK_F2: "f2",
        win32con.VK_F3: "f3",
        win32con.VK_F4: "f4",
        win32con.VK_F5: "f5",
        win32con.VK_F6: "f6",
        win32con.VK_F7: "f7",
        win32con.VK_F8: "f8",
        win32con.VK_F9: "f9",
        win32con.VK_F10: "f10",
        win32con.VK_F11: "f11",
        win32con.VK_F12: "f12",
        win32con.VK_DELETE: "del",
        0xC0: "backtick",
        0x30: "zero",
        0x31: "one",
        0x32: "two",
        0x33: "three",
        0x34: "four",
        0x35: "five",
        0x36: "six",
        0x37: "seven",
        0x38: "eight",
        0x39: "nine",
        0xBD: "minus",
        0xBB: "equals",
        win32con.VK_BACK: "backspace",
        win32con.VK_TAB: "tab",
        0x51: "q",
        0x57: "w",
        0x45: "e",
        0x52: "r",
        0x54: "t",
        0x59: "y",
        0x55: "u",
        0x49: "i",
        0x4F: "o",
        0x50: "p",
        0x41: "a",
        0x53: "s",
        0x44: "d",
        0x46: "f",
        0x47: "g",
        0x48: "h",
        0x4A: "j",
        0x4B: "k",
        0x4C: "l",
        0xBA: "semicolon",
        0xDE: "apostrophe",
        win32con.VK_LSHIFT: "leftShift",
        0x5A: "z",
        0x58: "x",
        0x43: "c",
        0x56: "v",
        0x42: "b",
        0x4E: "n",
        0x4D: "m",
        0xBC: "comma",
        0xBE: "period",
        0xBF: "forwardSlash",
        win32con.VK_RSHIFT: "rightShift",
        win32con.VK_LCONTROL: "leftCtrl",
        win32con.VK_LMENU: "leftAlt",
        win32con.VK_RMENU: "rightAlt",
        win32con.VK_RCONTROL: "rightCtrl",
        win32con.VK_LEFT: "left",
        win32con.VK_RIGHT: "right",
        win32con.VK_UP: "up",
        win32con.VK_DOWN: "down",
        win32con.VK_NUMPAD0: "numpad0",
        win32con.VK_NUMPAD1: "numpad1",
        win32con.VK_NUMPAD2: "numpad2",
        win32con.VK_NUMPAD3: "numpad3",
        win32con.VK_NUMPAD4: "numpad4",
        win32con.VK_NUMPAD5: "numpad5",
        win32con.VK_NUMPAD6: "numpad6",
        win32con.VK_NUMPAD7: "numpad7",
        win32con.VK_NUMPAD8: "numpad8",
        win32con.VK_NUMPAD9: "numpad9",
    }
    def on_press(key):
        print(key)

    while True:
        for key in keys:
            table.putBoolean(keys[key], win32api.GetAsyncKeyState(key))

if __name__ == '__main__':
    main()








