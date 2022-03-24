#! /usr/bin/env python

"""
user:~$ rossrv show gazebo_msgs/GetModelState
string model_name
string relative_entity_name
---
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
bool success
string status_message
"""

"""
rosmsg show GripperCommand
[control_msgs/GripperCommand]:
float64 position
float64 max_effort
"""

"""
rosmsg show TransformStamped
[geometry_msgs/TransformStamped]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
# import tf
import tf2_ros
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult

class PickPlace():

    HOME_TARGET = "home"
    AWAY_TARGET = "away"
    GRAB_TARGET = "grab"

    WAIT_BETWEEN_COMMANDS = 3.0

    GRIPPER_RELEASE_POSITION = 0.75
    GRIPPER_GRIP_POSITION = -1 * GRIPPER_RELEASE_POSITION
    GRIPPER_MAX_EFFORT = 10.0

    DEMO_CUBE = "demo_cube"

    MODEL_STATE_SERVICE = '/gazebo/get_model_state'

    GRIPPER_ACTION_SERVER = '/gripper_controller/gripper_cmd'

    GRASPABLE_OBJECT_PARENT_FRAME = "world"
    GRASPABLE_OBJECT = "graspable_object"
    
    def __init__(self):
        rospy.wait_for_service(PickPlace.MODEL_STATE_SERVICE)
        model_state_service = rospy.ServiceProxy(PickPlace.MODEL_STATE_SERVICE, GetModelState)
        model_state_request = GetModelStateRequest()
        model_state_request.model_name = PickPlace.DEMO_CUBE
        model_state_response = model_state_service(model_state_request)
        self._demo_cube_position = model_state_response.pose.position
        q = model_state_response.pose.orientation
        """
        self._demo_cube_quaternion = q
        self._demo_cube_euler = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        rospy.loginfo(f"{self._demo_cube_position=} {self._demo_cube_euler=}")
        """
        self._tf_Buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_Buffer)

        self._wait_between_commands = rospy.Rate(PickPlace.WAIT_BETWEEN_COMMANDS)

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface() 
        self._group = moveit_commander.MoveGroupCommander("arm")

        
        self._gripper_as_client = actionlib.SimpleActionClient(
            PickPlace.GRIPPER_ACTION_SERVER, GripperCommandAction)
        rospy.loginfo(f"Waiting for {PickPlace.GRIPPER_ACTION_SERVER}")
        self._gripper_as_client.wait_for_server()
        rospy.loginfo(f"{PickPlace.GRIPPER_ACTION_SERVER} is ready")

        self._ctrl_c = False
        rospy.on_shutdown(self.__shutdownhook)

    def __move_to_graspable_object(self):
        try:
            # transform_stamped = self._tf_Buffer.lookup_transform(PickPlace.GRASPABLE_OBJECT, PickPlace.GRASPABLE_OBJECT_PARENT_FRAME, rospy.Time(0))
            
            pose = Pose()
            """
            pose.orientation = transform_stamped.transform.rotation
            pose.position = transform_stamped.transform.translation
            """
            pose = self._group.get_current_pose().pose
            rospy.loginfo(f"{self._group.get_end_effector_link()}")
            pose.position.z += 0.02
            self._group.set_pose_target(pose)
            self._group.plan()
            self._group.go(wait=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Unable to lookup {PickPlace.GRASPABLE_OBJECT}")

    def __moveToNamedTarget(self, target):
        rospy.loginfo(f"Going to named target {target}")
        self._group.set_named_target(target)
        self._group.plan()
        self._group.go(wait=True)
        rospy.loginfo(f"Went to named target {target}")
        self._wait_between_commands.sleep()
        

    def __gripper_to_position(self, position):
        max_effort = PickPlace.GRIPPER_MAX_EFFORT
        rospy.loginfo(f"Changing gripper position: {position=} {max_effort=}")
        gripper_command_goal = GripperCommandGoal()
        rospy.logdebug(f"{gripper_command_goal=}")
        gripper_command_goal.command.position = position
        gripper_command_goal.command.max_effort = max_effort
        self._gripper_as_client.send_goal(gripper_command_goal)
        rospy.logdebug(f"Sent {gripper_command_goal=}")
        rospy.loginfo("Waiting for result")
        self._gripper_as_client.wait_for_result()
        gripper_command_result = self._gripper_as_client.get_result()
        rospy.loginfo(f"{gripper_command_result=}")
        rospy.loginfo("Grip complete")
        self._wait_between_commands.sleep()


    def execute(self):
        self.__moveToNamedTarget(PickPlace.HOME_TARGET)

        self.__moveToNamedTarget(PickPlace.GRAB_TARGET)
        self.__move_to_graspable_object()

        """
        self.__gripper_to_position(PickPlace.GRIPPER_GRIP_POSITION)
        self.__moveToNamedTarget(PickPlace.AWAY_TARGET)
        self.__gripper_to_position(PickPlace.GRIPPER_RELEASE_POSITION)
        """
        moveit_commander.roscpp_shutdown()
        
    def __shutdownhook(self):
        self.ctrl_c = True
        moveit_commander.roscpp_shutdown()
            
if __name__ == '__main__':
    rospy.init_node("pick_place", anonymous=True, log_level=rospy.DEBUG)
    pick_place = PickPlace()
    pick_place.execute()
