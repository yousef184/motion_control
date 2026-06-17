import json
import time
import paho.mqtt.client as mqtt


# =========================
# MQTT CONFIGURATION
# =========================

BROKER_IP = "172.22.222.238"
BROKER_PORT = 1883

ROBOT_NAME = "cat001"
STATION_NAME = "station001"


# =========================
# TRANSFER SETTINGS
# =========================

# Drop: box moves from robot conveyor to station conveyor
# If direction is wrong, change +0.15 to -0.15
ROBOT_DROP_SPEED = 0.35
STATION_DROP_SPEED = 0.35

MAX_TRANSFER_TIME = 15.0  # safety timeout in seconds


# =========================
# GLOBAL SENSOR STATES
# =========================

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


# =========================
# MQTT CALLBACKS
# =========================

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


# =========================
# CONVEYOR COMMAND FUNCTIONS
# =========================

def set_conveyor_speed(client, name, value):
    topic = f"KIT/IMRL/{name}/conveyor/speed"
    payload = {"value": float(value)}

    client.publish(topic, json.dumps(payload))
    print(f"Published to {topic}: {payload}")


def stop_all(client):
    print("Stopping all conveyors...")
    set_conveyor_speed(client, ROBOT_NAME, 0.0)
    set_conveyor_speed(client, STATION_NAME, 0.0)


# =========================
# DROP ACTION
# =========================

def drop_robot_to_station(client):
    """
    Drop action:
    Box starts on robot conveyor.
    Conveyors move the box from robot to station.

    Stop condition:
    1. Box reaches station sensor A: station_a becomes True
    2. Box leaves station sensor A: station_a becomes False again
    3. Then stop both conveyors
    """

    print("\n==============================")
    print("DROP ACTION STARTED")
    print("==============================")
    print("Put the box on the robot conveyor.")
    print("Transfer direction: Robot -> Station")
    print("The system will stop when the box leaves station sensor A.")
    print("Starting in 2 seconds...")
    time.sleep(2)

    start_time = time.time()
    station_a_was_triggered = False

    while time.time() - start_time < MAX_TRANSFER_TIME:
        # Keep sending speed commands during the transfer
        set_conveyor_speed(client, ROBOT_NAME, ROBOT_DROP_SPEED)
        set_conveyor_speed(client, STATION_NAME, STATION_DROP_SPEED)

        station_a = station_state.get("sensor_a", False)
        station_b = station_state.get("sensor_b", False)
        robot_a = robot_state.get("sensor_a", False)
        robot_b = robot_state.get("sensor_b", False)

        print(
            "Robot A:", robot_a,
            "Robot B:", robot_b,
            "| Station A:", station_a,
            "Station B:", station_b,
        )

        # Step 1: detect when box first reaches station sensor A
        if station_a and not station_a_was_triggered:
            station_a_was_triggered = True
            print("Box reached station sensor A. Waiting until it leaves sensor A...")

        # Step 2: stop only after the box leaves station sensor A
        if station_a_was_triggered and not station_a:
            print("Box left station sensor A. Stopping transfer.")
            break

        time.sleep(0.3)

    if not station_a_was_triggered:
        print("WARNING: Station sensor A was not triggered before timeout.")

    stop_all(client)
    print("DROP ACTION FINISHED")


# =========================
# MAIN PROGRAM
# =========================

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    print("Connecting to MQTT broker:", BROKER_IP)
    client.connect(BROKER_IP, BROKER_PORT, 60)

    client.loop_start()

    try:
        # Wait to receive first sensor states
        time.sleep(2)

        # Run drop behavior
        drop_robot_to_station(client)

    except KeyboardInterrupt:
        print("\nStopped by user using Ctrl+C.")

    finally:
        stop_all(client)
        client.loop_stop()
        client.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()