# Formant ROS package

This ROS package contains examples for integrating with the Formant agent in a ROS context. Examples rely on the [`formant`](https://pypi.org/project/formant/) python pip package which can be installed with:

```bash
pip install formant
```

## Setup

The examples rely on using the [Formant turtlebot container](https://hub.docker.com/r/formant/turtlebot). This should provide us with a standard set of ROS topics and data to show a few examples.

First, make sure you have [docker](https://docs.docker.com/get-docker/) installed.

Run with:

```bash
 docker run -it --net host -a stderr -a stdin -a stdout -e TELEOP=1 -e SLAM=1 formant/turtlebot
```

Next, follow these [steps](https://help.formant.io/device-management/install-the-first-device) to install the Formant agent on your system.

Clone this repository to your `src` folder as another ROS package:

```
git clone https://github.com/formant-ros
```

After, building and sourcing your `setup.sh` file, you should be able to launch the formant node at this point with:

```bash
roslaunch formant main.launch
```

And you will see a ROS node spinning.

You'll want to keep this window around so we can drive the turtlebot to get some `/cmd_vel` messages.

## Examples

All examples are coded into our example packages [`main.py`](src/main.py). They also depend on setting up the Formant agent Client:

```python
self._formant_client = FormantClient(
    agent_url="localhost:5501", ignore_throttled=True, ignore_unavailable=True
)
```

### Topic Callbacks

One of the more traditional use cases is to subscribe to a ROS topic, perform either a message extraction or transformation on the data and then forward to the Formant agent. In this example we'll extract joint positional data and send as a numeric stream.

The first step is subscribing:

```python
rospy.Subscriber("/joint_states", JointState, self._joint_states_callback, queue_size=10)
```

Next we'll define our callback function:

```python
def _joint_states_callback(self, msg):
    """
    Integration with sensor_msgs/JointState.
    The turtlebot has two joints 'wheel_right_joint' and 'wheel_left_joint'.
    This function parses joint states into Formant Numeric streams
    which are sent to the agent.
    """
    joint_range = range(len(msg.name))
    for i in joint_range:
        # for each joint post to the numeric stream "wheel_joint_position"
        # with a tag for the joint
        self._formant_client.post_numeric(
            "wheel_joint_position", msg.position[i], tags={"joint": msg.name[i]}
        )
```

You can see here we iterate over the joints in the message and send along to the agent as a numeric stream. We can also transform the data to check if the position is > 0 and send as a bool:

```python
joint_state_positive = {}
joint_range = range(len(msg.name))
for i in joint_range:
    # set the state for each joint in the dict
    joint_state_positive[msg.name[i]] = msg.position[i] > 0
# send the joint state
self._formant_client.post_bitset(
    "wheel_joint_position_state_positive", joint_state_positive
)
```

This visualizes as a Formant bitset.

You can also go back to the window and use your keyboard to drive the turtlebot around to see the values here change in realtime on the Formant observability dashboard.

### Timer Callback

Another effective way to send data to formant is through a `rospy` timer. This allows us to periodically call a function on a set interval to deliver data.

The first step is setting up our timer:

```python
rospy.Timer(rospy.Duration(1), self._capture_state)
```

Then let's capture a few piecies of "state" in our callback.

```python
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
```

This type of implementation is useful for capturing stateful data in your application that may or may not be available via ROS topics.
