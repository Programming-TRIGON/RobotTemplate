import ntcore
import time
from pynput import keyboard

def main():
    ntcoreinst = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client")
    ntcoreinst.startClient4("KeyboardToNT")
    ntcoreinst.setServer("127.0.0.1")
    ntcoreinst.startDSClient()

    print("Waiting for connection to NetworkTables server...")
    while not ntcoreinst.isConnected():
        time.sleep(0.1)

    print("Connected!")
    table = ntcoreinst.getTable("SmartDashboard/keyboard")

    def on_press(key):
        handle_key(key, True)

    def on_release(key):
        handle_key(key, False)

    def handle_key(key, is_down):
        try:
            # Character keys
            if key.char == "/":
                return
            table.putBoolean(key.char.lower(), is_down)
        except AttributeError:
            # Special keys (shift, space, arrows, etc.)
            table.putBoolean(str(key).replace("Key.", ""), is_down)

    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    ) as listener:
        listener.join()

if __name__ == "__main__":
    main()
