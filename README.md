# cp_ur3e_moveit_config

## Part 7: Integrate everything

Start `gazebo` **twice** in order to for `/gripper_controller/gripper_cmd` to actually start up

```
source ~/simulation_ws/devel/setup.bash
roslaunch rb1_base_gazebo warehouse.launch
```

Don't move forward until the following command is successful

```
user:~$ rostopic list | grep gripper
/gripper_controller/gripper_cmd/cancel
/gripper_controller/gripper_cmd/feedback
/gripper_controller/gripper_cmd/goal
/gripper_controller/gripper_cmd/result
/gripper_controller/gripper_cmd/status
```

Use MoveIt to get the arm out of the way (go to the **away** position using the **Planning** tab)

```
roslaunch cp_ur3e_moveit_config cp_ur3e_planning_execution.launch
```

Spawn your camera into the simulation:

```
source ~/catkin_ws/devel/setup.bash
roslaunch box_camera_description camera_spawn.launch
```

Publish the **static transform** for the camera

```
source ~/catkin_ws/devel/setup.bash
roslaunch box_camera_description static_transform.launch
```

Launch **surface detection**

```
roslaunch project_object_detection surface_detection.launch
```

Get graspable object (publishes to /graspable_object_pose)

`roslaunch project_object_detection publish_object_position.launch`

Run `rviz` with `detection.rviz `from the `project_object_detection` package

Run

`rosrun cp_ur3e_moveit_config pick_place.py`
