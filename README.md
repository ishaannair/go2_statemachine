# go2_statemachine

A ROS2 package implementing a state machine for the Unitree Go2 robot, with optional LED status indication.

## Overview

The package manages robot operation states and transitions, integrates with Nav2 for autonomous navigation, and optionally drives a Circuit Playground Express LED ring to reflect the current state visually.

## States

| Index | State | Valid Transitions From |
|-------|-------|----------------------|
| 0 | `IDLE` | Any state |
| 1 | `ROBOT_MOVING` | IDLE |
| 2 | `IN_ACTION` | IDLE, ROBOT_MOVING |
| 3 | `PICK_OBJECT` | IDLE, ROBOT_MOVING |
| 4 | `EXTINGUISH_FIRE` | IDLE, ROBOT_MOVING |
| 5 | `DROP_OBJECT` | IDLE, ROBOT_MOVING |
| 6 | `COMPLETED_ACTION` | IN_ACTION, PICK_OBJECT, EXTINGUISH_FIRE, DROP_OBJECT, ROBOT_MOVING |
| 7 | `FAILED_ACTION` | IN_ACTION, PICK_OBJECT, EXTINGUISH_FIRE, DROP_OBJECT, ROBOT_MOVING |

Action states run for a fixed duration before automatically transitioning to `COMPLETED_ACTION`, then returning to `IDLE`:

| State | Duration |
|-------|----------|
| `IN_ACTION` | 10 s |
| `PICK_OBJECT` | 5 s |
| `EXTINGUISH_FIRE` | 10 s |
| `DROP_OBJECT` | 5 s |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/go2/set_state` | `std_msgs/Int8` | Subscribed | Request a state transition by index |
| `/go2/current_state` | `std_msgs/Int8` | Published | Current state index |
| `/go2/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | Subscribed | Nav2 goal status (drives ROBOT_MOVING) |

## Prerequisites

- ROS2 (Humble or later)
- `python3-transitions`

```bash
pip install transitions
```

## Installation

```bash
cd <your_ros2_ws>/src
git clone https://github.com/ishaannair/go2_statemachine.git
cd ..
colcon build --packages-select state_machine
source install/setup.bash
```

## Usage

### Launch (recommended)

```bash
# State machine only
ros2 launch state_machine state_machine.launch.py

# With LED indicator
ros2 launch state_machine state_machine.launch.py launch_led:=true

# With LED on a custom port
ros2 launch state_machine state_machine.launch.py launch_led:=true led_port:=/dev/ttyUSB0
```

#### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_led` | `false` | Set to `true` to start the LED indicator node |
| `led_port` | `/dev/ttyACM0` | Serial port for the Circuit Playground Express |
| `led_msg_type` | `int` | Message type for LED node (`int` or `string`) |

### Run nodes individually

```bash
ros2 run state_machine state_machine_node
ros2 run state_machine led_light_indicator_node --port /dev/ttyACM0 --msg-type int
```

### Trigger a state transition

```bash
# Request ROBOT_MOVING (index 1)
ros2 topic pub --once /go2/set_state std_msgs/msg/Int8 "data: 1"

# Request PICK_OBJECT (index 3)
ros2 topic pub --once /go2/set_state std_msgs/msg/Int8 "data: 3"
```

### Monitor current state

```bash
ros2 topic echo /go2/current_state
```

## LED Indicator

The `led_light_indicator_node` drives a Circuit Playground Express over USB serial and maps each state to a colour:

| State | Colour |
|-------|--------|
| IDLE | Blue |
| ROBOT_MOVING | Green |
| IN_ACTION | Cyan |
| PICK_OBJECT | Amber |
| EXTINGUISH_FIRE | Orange-red |
| DROP_OBJECT | Purple |
| COMPLETED_ACTION | Bright green |
| FAILED_ACTION | Red |

## Package Structure

```
go2_statemachine/
├── launch/
│   └── state_machine.launch.py
├── state_machine/
│   ├── __init__.py
│   ├── state_machine.py          # Main state machine node
│   └── led_light_indicator.py    # LED indicator node
├── resource/
│   └── state_machine
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```
