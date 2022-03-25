# cp_ur3e_moveit_config

## Part 7: Integrate everything

### Step 1

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

### Step 2

Sim
```
source ~/simulation_ws/devel/setup.bash
roslaunch rb1_base_gazebo warehouse_rb1.launch
```

Start the node in charge of performing the final attach to cart step (it will pause until the attach_to_cart parameter is set to true)

`roslaunch move_robot attach_to_cart.launch`

Move the `rb1`
```
rosrun attach_to_cart move_rb1_near_dock.py
```


Open RVIZ using the `attach_to_cart/rviz_config/metal_plates.rviz` configuration

Run

```
roslaunch attach_to_cart detect_cart.launch
```

Activate detect cart

`rosparam set /detect_cart "true"`

Run the script for aligning and attaching to the cart

`rosrun attach_to_cart attach_to_cart.py`

teleop if needed

```
source /opt/ros/noetic/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel
```
