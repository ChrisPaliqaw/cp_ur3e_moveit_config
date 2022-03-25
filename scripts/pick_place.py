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

    WAIT_BETWEEN_COMMANDS = 4.0

    GRIPPER_RELEASE_POSITION = 0.75
    GRIPPER_GRIP_POSITION = -1 * GRIPPER_RELEASE_POSITION
    GRIPPER_MAX_EFFORT = 10.0

    DEMO_CUBE = "demo_cube"

    MODEL_STATE_SERVICE = '/gazebo/get_model_state'

    GRIPPER_ACTION_SERVER = '/gripper_controller/gripper_cmd'

    GRASPABLE_OBJECT_PARENT_FRAME = 'world'# 'base_link' # "world"
    GRASPABLE_OBJECT = "graspable_object"

    ADJUST_X = -5.019327136768296
    ADJUST_Y = 4.059796511223785
    
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

        self._home_pose = None
        self._grab_pose = None

        """
        [INFO] [1648211286.376362, 1731.904000]: set_z=0.32126392468749704
        [INFO] [1648211286.388484, 1731.905000]: set_orientation=x: -0.5366506632278824
        y: 0.5264509022492222
        z: -0.4613877837701542
        w: 0.4711441670717353
        [INFO] [1648211286.393404, 1731.906000]: adjust_x=-5.019327136768296
        [INFO] [1648211286.405967, 1731.907000]: adjust_y=4.059796511223785
        """

        self._grab_pose = Pose()
        self._grab_pose.position.z = 0.3211662958172352
        self._grab_pose.orientation.x = -0.5367037404604901
        self._grab_pose.orientation.y = 0.5265342827156797
        self._grab_pose.orientation.z = -0.46133084299189914
        self._grab_pose.orientation.w = 0.4710462794728212

        graspable_transform = self.__get_graspable_transform()
        rospy.loginfo(f"{graspable_transform=}")
        self._grab_pose.position.x = graspable_transform.translation.x + PickPlace.ADJUST_X
        self._grab_pose.position.y = graspable_transform.translation.y + PickPlace.ADJUST_Y

        rospy.loginfo(f"{self._group.get_pose_reference_frame()=}")

    def __move_to_pose(self, pose: Pose):
        # transform_stamped = self._tf_Buffer.lookup_transform(PickPlace.GRASPABLE_OBJECT, PickPlace.GRASPABLE_OBJECT_PARENT_FRAME, rospy.Time(0))
        
        # pose = Pose()
        """
        pose.orientation = transform_stamped.transform.rotation
        pose.position = transform_stamped.transform.translation
        """
        # pose = self._group.get_current_pose().pose
        # rospy.loginfo(f"{self._group.get_end_effector_link()}")
        # pose.position.z += 0.02
        self._group.set_pose_target(pose)
        rospy.loginfo(f"Moving to {pose=}")
        self._group.plan()
        self._group.go(wait=True)
        self._wait_between_commands.sleep()
    
    def __get_graspable_transform(self):
        # self._tf_Buffer.wait_for_transform(PickPlace.GRASPABLE_OBJECT, PickPlace.GRASPABLE_OBJECT_PARENT_FRAME, rospy.Time())
        transform_stamped = self._tf_Buffer.lookup_transform(
            PickPlace.GRASPABLE_OBJECT_PARENT_FRAME, PickPlace.GRASPABLE_OBJECT, rospy.Time(), rospy.Duration(20.0))
        rospy.loginfo(f"Raw graspable tf is {transform_stamped}")
        return transform_stamped.transform

    def __move_to_gripping_pose(self):
        rospy.loginfo(f"Goal grip pose is {self._grab_pose}")
        # pose.position.z += PickPlace.Z_OFFSET_FROM_GRASPABLE_OBJECT
        self.__move_to_pose( self._grab_pose)

    def __move_to_named_target(self, target):
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
        self.__move_to_named_target(PickPlace.HOME_TARGET)
        # self.__move_to_named_target(PickPlace.GRAB_TARGET)
        self.__move_to_gripping_pose()
        self.__gripper_to_position(PickPlace.GRIPPER_GRIP_POSITION)
        self.__move_to_named_target(PickPlace.AWAY_TARGET)
        self.__gripper_to_position(PickPlace.GRIPPER_RELEASE_POSITION)
        rospy.spin()
        """
        
        rospy.sleep(20)

        self._grab_pose = self._group.get_current_pose().pose
        rospy.loginfo(f"{self._group.get_current_pose().pose=}")

        transform_stamped = self._tf_Buffer.lookup_transform(
            PickPlace.GRASPABLE_OBJECT_PARENT_FRAME, self._group.get_end_effector_link(), rospy.Time(), rospy.Duration(20.0))
        rospy.loginfo(f"The looked up transform is {transform_stamped}")
        """

        """
        set_z = self._grab_pose.position.z
        rospy.loginfo(f"{set_z=}")
        set_orientation = self._grab_pose.orientation
        rospy.loginfo(f"{set_orientation=}")
        adjust_x = self._grab_pose.position.x - transform_stamped.transform.translation.x
        rospy.loginfo(f"{adjust_x=}")
        adjust_y = self._grab_pose.position.y - transform_stamped.transform.translation.y
        rospy.loginfo(f"{adjust_y=}")
        """

        moveit_commander.roscpp_shutdown()
        
    def __shutdownhook(self):
        self.ctrl_c = True
        moveit_commander.roscpp_shutdown()
            
if __name__ == '__main__':
    rospy.init_node("pick_place", anonymous=True, log_level=rospy.DEBUG)
    pick_place = PickPlace()
    pick_place.execute()
