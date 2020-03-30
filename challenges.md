# Open Challenges

### Control

- Communication between Arduino running motor control and the Pi running ROS.
- Control of ROS slave by ROS master by communicating from PC to Pi.
- The definition of motor control and camera nodes.

### Track Identification

Heuristics:

- What constitutes a track?
- Is there a concise way to classify what a track is?

### Track Navigation

Assumptions:

- High contrast visual edge
- Left and right edges
- Track does not intersect
- Track width is not constant

Execution:

- Camera will capture track in front of robot

Driving optimization heuristics:

- The track edge must not be crossed
- Braking in a straight line is favored over braking during a turn
- The smallest delta in velocity is preferred
- The maximum velocity is preferred

