import json
import time
import paho.mqtt.client as mqtt


# ============================================================
# MQTT CONFIGURATION
# ============================================================

BROKER_IP = "172.22.222.238"
BROKER_PORT = 1883

ROBOT_NAME = "cat001"
STATION_NAME = "station001"


# ============================================================
# ACTION SELECTION
# ============================================================
# Use:
# ACTION = "drop"  -> robot to station
# ACTION = "pick"  -> station to robot

ACTION = "drop"


# ============================================================
# SPEED SETTINGS
# Recommended range: -0.2 to +0.2
# ============================================================

# DROP: robot -> station
DROP_ROBOT_SPEED = 0.25
DROP_STATION_SPEED = 0.25

# PICK: station -> robot
# We keep pick slower because box was falling before.
PICK_ROBOT_SPEED = -0.20
PICK_STATION_SPEED = -0.20

# Slow mode after robot sensor A is reached
PICK_SLOW_ROBOT_SPEED = -0.15
PICK_SLOW_STATION_SPEED = -0.15

MAX_TRANSFER_TIME = 20.0


# ============================================================
# GLOBAL SENSOR STATES
# ============================================================

robot_state = {
    "running": False,
    "sensor_a": False,
    "sensor_b": False,
}

station_state = {
    "running": False,
    "sensor_a": False,
    "sensor_b": False,
}


# ============================================================
# MQTT CALLBACKS
# ============================================================

def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected:", reason_code)

    robot_state_topic = f"KIT/IMRL/{ROBOT_NAME}/conveyor/state"
    station_state_topic = f"KIT/IMRL/{STATION_NAME}/conveyor/state"

    client.subscribe(robot_state_topic)
    client.subscribe(station_state_topic)

    print("Subscribed to conveyor states:")
    print(" -", robot_state_topic)
    print(" -", station_state_topic)
    print("Robot:", ROBOT_NAME)
    print("Station:", STATION_NAME)


def on_message(client, userdata, msg):
    global robot_state, station_state

    try:
        payload = json.loads(msg.payload.decode())

        if msg.topic == f"KIT/IMRL/{ROBOT_NAME}/conveyor/state":
            robot_state = payload

        elif msg.topic == f"KIT/IMRL/{STATION_NAME}/conveyor/state":
            station_state = payload

    except Exception as error:
        print("Error reading MQTT message:", error)
        print("Topic:", msg.topic)
        print("Raw payload:", msg.payload)


# ============================================================
# BASIC CONVEYOR FUNCTIONS
# ============================================================

def set_conveyor_speed(client, name, value):
    topic = f"KIT/IMRL/{name}/conveyor/speed"
    payload = {"value": float(value)}

    client.publish(topic, json.dumps(payload))
    print(f"Published to {topic}: {payload}")


def stop_all(client):
    set_conveyor_speed(client, ROBOT_NAME, 0.0)
    set_conveyor_speed(client, STATION_NAME, 0.0)


def stop_all_repeated(client, repeat_count=5, delay=0.1):
    print("Stopping all conveyors...")
    for _ in range(repeat_count):
        stop_all(client)
        time.sleep(delay)


def print_sensor_states():
    robot_a = robot_state.get("sensor_a", False)
    robot_b = robot_state.get("sensor_b", False)
    station_a = station_state.get("sensor_a", False)
    station_b = station_state.get("sensor_b", False)

    print(
        "Robot A:", robot_a,
        "Robot B:", robot_b,
        "| Station A:", station_a,
        "Station B:", station_b,
    )


# ============================================================
# DROP ACTION: ROBOT -> STATION
# ============================================================

def drop_robot_to_station(client):
    """
    Drop action:
    Box starts on robot conveyor.
    Box moves from robot conveyor to station conveyor.

    Stop logic:
    1. Wait until station sensor A becomes True.
    2. Then wait until station sensor A becomes False again.
    3. Stop conveyors.
    """

    print("\n==============================")
    print("DROP ACTION STARTED")
    print("==============================")
    print("Put the box on the ROBOT conveyor.")
    print("Transfer direction: Robot -> Station")
    print("Stop condition: station sensor A True -> False")
    print("Starting in 2 seconds...")
    time.sleep(2)

    start_time = time.time()
    station_a_was_triggered = False

    while time.time() - start_time < MAX_TRANSFER_TIME:
        station_a = station_state.get("sensor_a", False)
        station_b = station_state.get("sensor_b", False)
        robot_a = robot_state.get("sensor_a", False)
        robot_b = robot_state.get("sensor_b", False)

        # Step 1: box reaches station sensor A
        if station_a and not station_a_was_triggered:
            station_a_was_triggered = True
            print("Box reached station sensor A. Waiting until it leaves station sensor A...")

        # Step 2: box leaves station sensor A
        if station_a_was_triggered and not station_a:
            print("Box left station sensor A. Stopping drop transfer.")
            break

        set_conveyor_speed(client, ROBOT_NAME, DROP_ROBOT_SPEED)
        set_conveyor_speed(client, STATION_NAME, DROP_STATION_SPEED)

        print(
            "Robot A:", robot_a,
            "Robot B:", robot_b,
            "| Station A:", station_a,
            "Station B:", station_b,
        )

        time.sleep(0.2)

    if not station_a_was_triggered:
        print("WARNING: Station sensor A was not triggered before timeout.")

    stop_all_repeated(client)
    print("DROP ACTION FINISHED")


# ============================================================
# PICK ACTION: STATION -> ROBOT
# ============================================================

def pick_station_to_robot(client):
    """
    Pick action:
    Box starts on station conveyor.
    Box moves from station conveyor to robot conveyor.

    Stop logic:
    1. Wait until robot sensor B becomes True.
    2. Then wait until robot sensor B becomes False again.
    3. Stop conveyors.
    """

    print("\n==============================")
    print("PICK ACTION STARTED")
    print("==============================")
    print("Put the box on the STATION conveyor.")
    print("Transfer direction: Station -> Robot")
    print("Stop condition: robot sensor B True -> False")
    print("Starting in 2 seconds...")
    time.sleep(2)

    start_time = time.time()
    robot_b_was_triggered = False

    while time.time() - start_time < MAX_TRANSFER_TIME:
        robot_a = robot_state.get("sensor_a", False)
        robot_b = robot_state.get("sensor_b", False)
        station_a = station_state.get("sensor_a", False)
        station_b = station_state.get("sensor_b", False)

        # Step 1: box reaches robot sensor B
        if robot_b and not robot_b_was_triggered:
            robot_b_was_triggered = True
            print("Box reached robot sensor B. Waiting until it leaves robot sensor B...")

        # Step 2: box leaves robot sensor B
        if robot_b_was_triggered and not robot_b:
            print("Box left robot sensor B. Stopping pick transfer.")
            break

        set_conveyor_speed(client, ROBOT_NAME, PICK_ROBOT_SPEED)
        set_conveyor_speed(client, STATION_NAME, PICK_STATION_SPEED)

        print(
            "Robot A:", robot_a,
            "Robot B:", robot_b,
            "| Station A:", station_a,
            "Station B:", station_b,
        )

        time.sleep(0.1)

    if not robot_b_was_triggered:
        print("WARNING: Robot sensor B was not triggered before timeout.")

    stop_all_repeated(client)
    print("PICK ACTION FINISHED")

# ============================================================
# MAIN PROGRAM
# ============================================================

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    print("Connecting to MQTT broker:", BROKER_IP)
    client.connect(BROKER_IP, BROKER_PORT, 60)

    client.loop_start()

    try:
        # Wait for first sensor states
        time.sleep(2)

        if ACTION == "drop":
            drop_robot_to_station(client)

        elif ACTION == "pick":
            pick_station_to_robot(client)

        else:
            print("Invalid ACTION. Use 'drop' or 'pick'.")

    except KeyboardInterrupt:
        print("\nStopped by user using Ctrl+C.")

    finally:
        stop_all_repeated(client)
        client.loop_stop()
        client.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()