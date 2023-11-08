from typing import List

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


########################################################################################################################
class StretchManipulation:
    def __init__(self):
        self.trajectory_client = actionlib.SimpleActionClient(
            "/stretch_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        server_reached = self.trajectory_client.wait_for_server(timeout=rospy.Duration(15.0))
        if not server_reached:
            rospy.logerr("Failed to connect to the trajectory server.")
            return
        
        rospy.loginfo("<CHECK> {0}: Made contact with trajectory server".format(self.__class__.__name__))
        self.trajectory_goal = FollowJointTrajectoryGoal()
        self.point0 = JointTrajectoryPoint()

    # Home the robot for the first time after bootup
    def trigger_home_the_robot(self):
        rospy.wait_for_service("/calibrate_the_robot")
        try:
            trigger_homing = rospy.ServiceProxy("/calibrate_the_robot", Trigger)
            response = trigger_homing()
            return response.message
        except rospy.ServiceException as e:
            rospy.logerr(f"Home the robot service call failed: {e}")
            return None
    
    def send_joint_goals(self, joint_names: List[str], goal_values: List[float]):
        if len(joint_names) != len(goal_values):
            rospy.logwarn("Joint names and goal values lists must have the same length.")
            return

        self.trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        self.trajectory_goal.trajectory.header.frame_id = "base_link" # relative to robot base
        self.trajectory_goal.trajectory.joint_names = joint_names
        self.point0.positions = goal_values
        self.trajectory_goal.trajectory.points = [self.point0]
        
        self.trajectory_client.send_goal(self.trajectory_goal)
        rospy.loginfo("Sent goal = {0}".format(self.trajectory_goal))
        self.trajectory_client.wait_for_result()

    def gripper_close(self):
        rospy.loginfo("{0}: Closing gripper".format(self.__class__.__name__))
        self.send_joint_goals(['joint_gripper_finger_left'], [0.165])


    def gripper_open(self):
        rospy.loginfo("{0}: Opening gripper".format(self.__class__.__name__))
        self.send_joint_goals(["joint_gripper_finger_left"], [-0.35])


    def lift_up(self):
        rospy.loginfo("{0}: Lifting arm up".format(self.__class__.__name__))
        self.send_joint_goals(["joint_lift"], [1.00])


    def reach_down(self):
        rospy.loginfo("{0}: Dropping arm down".format(self.__class__.__name__))
        self.send_joint_goals(["joint_lift"], [0.2])

# End of class
########################################################################################################################