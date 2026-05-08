#!/usr/bin/env python3
from transitions import Machine
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import rclpy
from std_msgs.msg import Int8
from action_msgs.msg import GoalStatusArray, GoalStatus
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import sys

OPERATION_STATES_AND_TRANSITIONS_DICT = {
    0: ['IDLE',             '*'],
    1: ['ROBOT_MOVING',     ['IDLE']],
    2: ['IN_ACTION',        ['IDLE', 'ROBOT_MOVING']],
    3: ['PICK_OBJECT',      ['IDLE', 'ROBOT_MOVING']],
    4: ['EXTINGUISH_FIRE',  ['IDLE', 'ROBOT_MOVING']],
    5: ['DROP_OBJECT',      ['IDLE', 'ROBOT_MOVING']],
    6: ['COMPLETED_ACTION', ['IN_ACTION', 'PICK_OBJECT', 'EXTINGUISH_FIRE', 'DROP_OBJECT', 'ROBOT_MOVING']],
    7: ['FAILED_ACTION',    ['IN_ACTION', 'PICK_OBJECT', 'EXTINGUISH_FIRE', 'DROP_OBJECT', 'ROBOT_MOVING']]
}

# Duration in seconds for each timed action state.
# IN_ACTION is the generic fallback for unregistered actions.
ACTION_TIMES = {
    'IN_ACTION':       10,
    'PICK_OBJECT':     5,
    'EXTINGUISH_FIRE': 10,
    'DROP_OBJECT':     5
}

allow_self_transitions = False
Development_phase = True


class SimpleStateMachine(Node):
    def __init__(self):
        super().__init__('simple_state_machine')
        self.get_logger().info("State Machine Python node initialized")
        topic_prefix = '/go2'
        set_state_topic = f'{topic_prefix}/set_state'
        current_state_topic = f'{topic_prefix}/current_state'
        operation_states, operation_transitions = self.parse_states_and_transitions(OPERATION_STATES_AND_TRANSITIONS_DICT)
        self.current_state_num = list(OPERATION_STATES_AND_TRANSITIONS_DICT.keys())[0]
        self.machine = Machine(
            model=self, states=operation_states, transitions=operation_transitions,
            initial=operation_states[0], auto_transitions=False)

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        if Development_phase:
            self.get_logger().info("Development phase is ON")
            self.create_subscription(Int8, current_state_topic, self.action_callback, 10)
            self.create_subscription(Int8, current_state_topic, self.post_action_callback, 10)

        self.current_state_pub = self.create_publisher(
            Int8, current_state_topic, qos_profile)

        # Subscribe to nav2 NavigateToPose action status to drive ROBOT_MOVING state.
        # Resolves to /{namespace}/navigate_to_pose/_action/status when namespaced.
        nav2_status_qos = QoSProfile(depth=10)
        nav2_status_qos.reliability = ReliabilityPolicy.RELIABLE
        nav2_status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            GoalStatusArray,
            '/go2/navigate_to_pose/_action/status',
            self.nav2_status_callback,
            nav2_status_qos)

        self.current_state_pub.publish(Int8(data=self.current_state_num))
        self.create_subscription(Int8, set_state_topic, self.set_state_callback, 10)

    def set_state_callback(self, msg: Int8):
        if msg.data not in list(OPERATION_STATES_AND_TRANSITIONS_DICT.keys()):
            self.get_logger().warn(f"⚠️  Invalid state index: {msg.data}")
            return

        req_state = OPERATION_STATES_AND_TRANSITIONS_DICT[msg.data][0]
        req_trigger = f'to_{req_state}'
        self.get_logger().info(f"Trying: {req_trigger}")
        try:
            self.trigger(req_trigger)
        except Exception as e:
            self.get_logger().warn(f"⚠️  {self.state} to {req_state} is an invalid state transition! ({e})")
        else:
            self.current_state_num = msg.data
            self.publish_state()

    def action_callback(self, msg: Int8):
        # Handle all timed action states: IN_ACTION, PICK_OBJECT, EXTINGUISH_FIRE, DROP_OBJECT
        action_index_map = {k: v[0] for k, v in OPERATION_STATES_AND_TRANSITIONS_DICT.items()
                            if v[0] in ACTION_TIMES}
        if msg.data not in action_index_map:
            return
        state_name = action_index_map[msg.data]
        action_time = ACTION_TIMES[state_name]
        self.get_logger().info(f"Performing {state_name} for {action_time} seconds")
        time.sleep(action_time)
        self.get_logger().info(f"{state_name} completed.")
        completed_idx = next(k for k, v in OPERATION_STATES_AND_TRANSITIONS_DICT.items()
                             if v[0] == 'COMPLETED_ACTION')
        self.current_state_num = completed_idx
        self.trigger('to_COMPLETED_ACTION')
        self.publish_state()

    def post_action_callback(self, msg: Int8):
        terminal_indices = {k for k, v in OPERATION_STATES_AND_TRANSITIONS_DICT.items()
                            if v[0] in ('COMPLETED_ACTION', 'FAILED_ACTION')}
        if msg.data not in terminal_indices:
            return
        state_name = OPERATION_STATES_AND_TRANSITIONS_DICT[msg.data][0]
        self.get_logger().info(f"Post action processing for {state_name}")
        time.sleep(2)
        self.get_logger().info("Returning to IDLE state.")
        self.current_state_num = 0
        self.trigger('to_IDLE')
        self.publish_state()

    def nav2_status_callback(self, msg: GoalStatusArray):
        """Tracks nav2 NavigateToPose goal status to manage ROBOT_MOVING state."""
        if not msg.status_list:
            return
        latest_status = msg.status_list[-1].status

        if latest_status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
            if self.state != 'ROBOT_MOVING':
                try:
                    self.trigger('to_ROBOT_MOVING')
                    moving_idx = next(k for k, v in OPERATION_STATES_AND_TRANSITIONS_DICT.items()
                                      if v[0] == 'ROBOT_MOVING')
                    self.current_state_num = moving_idx
                    self.publish_state()
                except Exception as e:
                    self.get_logger().debug(f"Cannot transition to ROBOT_MOVING: {e}")

        elif latest_status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == 'ROBOT_MOVING':
                try:
                    completed_idx = next(k for k, v in OPERATION_STATES_AND_TRANSITIONS_DICT.items()
                                         if v[0] == 'COMPLETED_ACTION')
                    self.current_state_num = completed_idx
                    self.trigger('to_COMPLETED_ACTION')
                    self.publish_state()
                except Exception as e:
                    self.get_logger().debug(f"Cannot transition ROBOT_MOVING -> COMPLETED_ACTION: {e}")

        elif latest_status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            if self.state == 'ROBOT_MOVING':
                try:
                    failed_idx = next(k for k, v in OPERATION_STATES_AND_TRANSITIONS_DICT.items()
                                      if v[0] == 'FAILED_ACTION')
                    self.current_state_num = failed_idx
                    self.trigger('to_FAILED_ACTION')
                    self.publish_state()
                except Exception as e:
                    self.get_logger().debug(f"Cannot transition ROBOT_MOVING -> FAILED_ACTION: {e}")

    def publish_state(self):
        msg = Int8()
        msg.data = self.current_state_num
        self.current_state_pub.publish(msg)
        self.get_logger().info(
            f"📤 Published current_state = {msg.data} ({self.state})")

    def parse_states_and_transitions(self, states_and_transitions_dict):
        operation_states_transitions = list(states_and_transitions_dict.values())
        operation_states = []
        operation_transitions = []
        for i, state_transitions_row in enumerate(operation_states_transitions):
            operation_states.append(state_transitions_row[0])
            if allow_self_transitions and type(state_transitions_row[1]) is list:
                state_transitions_row[1].append(operation_states[i])

            operation_transitions.append([])
            operation_transitions[i].append(f'to_{operation_states[i]}')
            operation_transitions[i].append(state_transitions_row[1])
            operation_transitions[i].append(operation_states[i])

        return (operation_states, operation_transitions)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()