"""
ROS2 node: subscribes to /go2/current_state and drives the Circuit Playground
Express LED ring via USB serial (COM6 by default).

Supported message types (set MSG_TYPE below):
  "int"    -> std_msgs/msg/Int32   (state as integer index 0-7)
  "string" -> std_msgs/msg/String  (state as name e.g. "IDLE")

Usage:
  python led_indicator_node.py
  python led_indicator_node.py --port /dev/ttyACM0 --msg-type int
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
import serial

# ── State definitions ──────────────────────────────────────────────────────────
# id -> (name, R, G, B)
STATES = {
    0: ("IDLE",             0,   0,   80),   # blue
    1: ("ROBOT_MOVING",     0,   80,  0),    # green
    2: ("IN_ACTION",        0,   60,  80),   # cyan
    3: ("PICK_OBJECT",      80,  50,  0),    # amber
    4: ("EXTINGUISH_FIRE",  80,  20,  0),    # orange-red
    5: ("DROP_OBJECT",      60,  0,   80),   # purple
    6: ("COMPLETED_ACTION", 0,   80,  20),   # bright green
    7: ("FAILED_ACTION",    80,  0,   0),    # red
}

NAME_TO_ID = {v[0]: k for k, v in STATES.items()}


def send_state(ser: serial.Serial, state_id: int) -> None:
    if state_id not in STATES:
        return
    name, r, g, b = STATES[state_id]
    command = f"{name},{r},{g},{b}\n"
    ser.write(command.encode("utf-8"))


class LedIndicatorNode(Node):
    def __init__(self, port: str, msg_type: str):
        super().__init__("led_indicator")

        try:
            self._ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # wait for CircuitPython boot
            self.get_logger().info(f"Serial open on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open {port}: {e}")
            sys.exit(1)

        self._last_id = -1

        if msg_type == "int":
            self._sub = self.create_subscription(
                Int8,
                "/go2/current_state",
                self._cb_int,
                10,
            )
        else:
            self._sub = self.create_subscription(
                String,
                "/go2/current_state",
                self._cb_string,
                10,
            )

        self.get_logger().info(
            f"Subscribed to /go2/current_state [{msg_type}]"
        )

        # Show startup color (IDLE)
        send_state(self._ser, 0)

    def _cb_int(self, msg: Int8) -> None:
        state_id = msg.data
        if state_id == self._last_id:
            return
        self._last_id = state_id
        name = STATES.get(state_id, (f"UNKNOWN({state_id})",))[0]
        self.get_logger().info(f"State -> {state_id}: {name}")
        send_state(self._ser, state_id)

    def _cb_string(self, msg: String) -> None:
        name = msg.data.strip().upper()
        state_id = NAME_TO_ID.get(name)
        if state_id is None:
            self.get_logger().warn(f"Unknown state name: '{name}'")
            return
        if state_id == self._last_id:
            return
        self._last_id = state_id
        self.get_logger().info(f"State -> {state_id}: {name}")
        send_state(self._ser, state_id)

    def destroy_node(self):
        send_state(self._ser, 0)   # return to IDLE color on shutdown
        self._ser.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",     default="/dev/ttyACM0")
    parser.add_argument("--msg-type", default="int", choices=["int", "string"])
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = LedIndicatorNode(args.port, args.msg_type)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
