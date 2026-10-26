# PAIA26: Language-directed, bounded multi-joint robot patrol (v0.3)

This is an original extension of the open-source RT Corporation CRANE+ V2 / MoveIt
examples. ROS2 Humble, Ignition Gazebo, and local Ollama are required.

## Architecture

Japanese instruction -> local Ollama Qwen -> validated list of stations
(LEFT/CENTER/RIGHT) -> MoveIt planning and collision checks -> simulated CRANE+
multi-joint extension/retraction -> measured joint verification -> JSONL log.

No image understanding or inspections have been implemented. Station names map
to fixed joint-space poses for a demonstration; they are not independent
perception of a physical target. The LLM cannot choose arbitrary angles.

## Motion targets

Every station moves the entire arm from the current state to
`vertical-with-base-yaw`, then `home-with-base-yaw`. The latter two multi-joint
pose families use the CRANE+ SRDF vertical and home configurations at yaw=0,
with fixed bounded yaw at LEFT=-0.40, CENTER=0, RIGHT=+0.40 rad.
Planning and execution are performed by MoveIt, with result checks.

## Build inside existing ROS2 container

`source /opt/ros/humble/setup.bash; source ~/airobot_ws/install/setup.bash`

`cd moveit_ws && colcon build --packages-select paia26_moveit`

`source moveit_ws/install/setup.bash`

## Run

Planning only (does not move robot):
`python3 moveit_demo.py --instruction '右、左、中央の順に巡回して'`

Executing (only inside configured simulation container):
`python3 moveit_demo.py --instruction '右、左、中央の順に巡回して' --execute`

Use environment PAIA_OLLAMA_URL, PAIA_OLLAMA_MODEL. Runtime source must be
included in a public repo; no secrets or personal identifiers.

## Third-party dependencies

MoveIt 2 and CRANE+ V2 ROS2 packages are used at runtime, not copied into this
repository. The official vendor `pose_groupstate` example was used to validate
the baseline multi-joint behavior. Cite vendor example and included licensing
when preparing the final GitHub repository.
