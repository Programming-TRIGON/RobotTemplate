import keyboard
import ntcore
import threading
import time

MINIMUM_PRESS_TIME = 0.35
RECONNECT_INTERVAL = 2.0

keys_lock = threading.Lock()
keys_dict = {}  # key -> [timestamp, release_pending]

table = None
table_lock = threading.Lock()


def get_table():
    with table_lock:
        return table


def set_table(t):
    global table
    with table_lock:
        table = t


def put_boolean_safe(key, value):
    t = get_table()
    if t is None:
        return False  # indicate not connected
    try:
        t.putBoolean(key, value)
        return True
    except Exception as e:
        print(f"NT write failed for {key}={value}: {e}")
        return False


def connect_nt():
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.stopClient()
    inst.startClient4("KeyboardToNT")
    inst.setServer("127.0.0.1")
    inst.startDSClient()
    return inst


def nt_watchdog():
    """Runs in background. Reconnects NT when disconnected, clears stale keys on reconnect."""
    inst = connect_nt()
    print("Waiting for initial NT connection...")
    was_connected = False

    while True:
        connected = inst.isConnected()

        if connected and not was_connected:
            print("NT connected.")
            # Flush stale key state — don't blast the server with old data
            with keys_lock:
                keys_dict.clear()
            set_table(inst.getTable("/SmartDashboard/keyboard"))

        elif not connected and was_connected:
            print("NT disconnected. Clearing table reference...")
            set_table(None)
            with keys_lock:
                keys_dict.clear()

        was_connected = connected
        time.sleep(0.5)


def turn_off_keys_with_delay():
    while True:
        time.sleep(0.01)
        to_remove = []

        with keys_lock:
            for key, (ts, release_pending) in list(keys_dict.items()):
                if release_pending and time.time() - ts > MINIMUM_PRESS_TIME:
                    to_remove.append(key)

        for key in to_remove:
            sent = put_boolean_safe(key, False)
            if sent:
                print(f"{key} released")  # only print if actually sent
            with keys_lock:
                keys_dict.pop(key, None)


def on_action(event: keyboard.KeyboardEvent):
    if event is None or event.name is None or event.name == "/":
        return

    key = ("numpad" + event.name) if event.is_keypad else event.name.lower()
    is_pressed = event.event_type == keyboard.KEY_DOWN

    if is_pressed:
        sent = put_boolean_safe(key, True)
        if sent:
            print(f"{key} pressed")  # only print if actually sent
        with keys_lock:
            if key not in keys_dict:
                keys_dict[key] = [time.time(), False]
    else:
        with keys_lock:
            if key in keys_dict:
                keys_dict[key][1] = True
            else:
                keys_dict[key] = [time.time(), True]


def on_action(event: keyboard.KeyboardEvent):
    if event is None or event.name is None or event.name == "/":
        return

    key = ("numpad" + event.name) if event.is_keypad else event.name.lower()
    is_pressed = event.event_type == keyboard.KEY_DOWN

    if is_pressed:
        put_boolean_safe(key, True)
        with keys_lock:
            if key not in keys_dict:
                keys_dict[key] = [time.time(), False]
    else:
        with keys_lock:
            if key in keys_dict:
                keys_dict[key][1] = True  # mark for delayed release
            else:
                keys_dict[key] = [time.time(), True]


def main():
    threading.Thread(target=nt_watchdog, daemon=True).start()
    threading.Thread(target=turn_off_keys_with_delay, daemon=True).start()

    keyboard.hook(on_action)
    print("Listening for keyboard input. Press Ctrl+C to exit.")
    keyboard.wait()


main()
