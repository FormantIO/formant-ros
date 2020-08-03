#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState

from formant.sdk.agent.v1 import Client as FormantClient


class FormantNode:
    def __init__(self):
        """
        Integration with Formant agent
        """
        rospy.init_node("formant_node")
        # Create ROS subscribers
        self._subscriptions = [
            rospy.Subscriber(
                "/joint_states", JointState, self._joint_states_callback, queue_size=10,
            ),
        ]
        # Ignore throttled or agent unavailable exceptions
        self._formant_client = FormantClient(
            agent_url="localhost:5501", ignore_throttled=True, ignore_unavailable=True
        )
        rospy.Timer(rospy.Duration(1), self._capture_state)
        rospy.spin()

    def _joint_states_callback(self, msg):
        """
        Integration with sensor_msgs/JointState.
        The turtlebot has two joints 'wheel_right_joint' and 'wheel_left_joint'.
        This function parses joint states into Formant Numeric streams
        which are sent to the agent.
        """
        joint_state_positive = {}
        joint_range = range(len(msg.name))
        for i in joint_range:
            # for each joint post to the numeric stream "wheel_joint_position"
            # with a tag for the joint
            self._formant_client.post_numeric(
                "wheel_joint_position", msg.position[i], tags={"joint": msg.name[i]}
            )
            # set the state for each joint in the dict
            joint_state_positive[msg.name[i]] = msg.position[i] > 0
        # send the joint state
        self._formant_client.post_bitset(
            "wheel_joint_position_state_positive", joint_state_positive
        )

    def _capture_state(self, event=None):
        """
        Use a timed callback method for data not available through topic callbacks, 
        or data that needs to be polled periodically.
        """
        # send the system state on a text stream
        self._formant_client.post_text("system_state.mode", "RUNNING")

        # send a bitset of the system state
        self._formant_client.post_bitset(
            "system_state",
            {"RUNNING": True, "STOPPED": False, "ERROR": False, "UNKNOWN": False},
        )


if __name__ == "__main__":
    try:
        FormantNode()
    except rospy.ROSInterruptException:
        pass
