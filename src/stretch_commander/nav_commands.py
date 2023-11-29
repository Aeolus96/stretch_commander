import logging
import sys
import time

import actionlib
import rospy
from alive_progress import alive_bar
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger
from stretch_srvs.srv import MoveArm, MoveArmRequest, MoveArmResponse
from tf import transformations


########################################################################################################################
class StretchNavigation:
    # Status bar configuration (Alive Bar): https://pypi.org/project/alive-progress/
    # NOTE: Nested bars are not supported right now
    bar_config = dict(
        bar=None,
        monitor=False,
        elapsed=False,
        elapsed_end=False,
        stats=False,
        receipt=True,
        receipt_text=False,
        enrich_print=False,
    )

    def __init__(self, frame_id="map"):
        # Set up the MoveBaseAction client and initialize sub/pubs
        with alive_bar(**self.bar_config) as bar:
            bar.text("Connecting to move_base...")
            self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

            if not self.client.wait_for_server(rospy.Duration(5)):
                bar.title("Connecting to move_base failed")
                rospy.logerr("Shutting down - failure to connect to the move_base server.")
                sys.exit(1)  # exit because Navigation wont work without a MoveBase server
            bar.title("move_base server connected")

            self.move_base_goal = MoveBaseGoal()
            self.move_base_goal.target_pose.header.frame_id = frame_id  # map frame for convenience
            # publisher for manipulation goal points
            self.clicked_point_pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=1)
            self.clicked_point_goal = PoseStamped()
            self.pick_up_point = Point()  # for MoveArm service

    # Re-locates the robot in the pre-loaded map
    def trigger_global_localization(self):
        with alive_bar(**self.bar_config) as bar:
            bar.text("Re-locating the robot...")
            rospy.wait_for_service("/funmap/trigger_global_localization")
            try:
                trigger_loc = rospy.ServiceProxy("/funmap/trigger_global_localization", Trigger)
                response = trigger_loc()
                bar.title("Re-locating the robot complete")
                return response.message
            except rospy.ServiceException as e:
                rospy.logerr(f"Global localization service call failed: {e}")
                return None

    # Scans the surrounding area using the RealSense depth camera
    def trigger_head_scan(self):
        with alive_bar(**self.bar_config) as bar:
            bar.text("Scanning the area...")
            rospy.wait_for_service("/funmap/trigger_head_scan")
            try:
                trigger_scan = rospy.ServiceProxy("/funmap/trigger_head_scan", Trigger)
                response = trigger_scan()
                bar.title("Scanning the area complete")
                return response.message
            except rospy.ServiceException as e:
                rospy.logerr(f"Head scan service call failed: {e}")
                return None

    # Converts an angle in radians to a quaternion for use with MoveBaseGoal
    def get_quaternion(self, theta):
        return Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))

    # Callback for when the goal is reached
    def done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"<CHECK> {self.__class__.__name__}: SUCCEEDED in reaching the goal.")
        else:
            rospy.loginfo(f"<CHECK> {self.__class__.__name__}: FAILED in reaching the goal.")
        return

    # x, y, and a are in meters and radians. Uses MoveBase Action Server directly
    # Use math.radians(theta) to use degrees
    def go_to_xya(self, x, y, theta, max_retries=3):
        with alive_bar(**self.bar_config) as bar:
            bar.text(f"Moving to ({x}m, {y}m) at {theta} radians")
            self.move_base_goal.target_pose.header.stamp = rospy.Time()
            self.move_base_goal.target_pose.pose.position.x = x
            self.move_base_goal.target_pose.pose.position.y = y
            self.move_base_goal.target_pose.pose.position.z = 0.0  # Ground plane as default
            self.move_base_goal.target_pose.pose.orientation = self.get_quaternion(theta)

            # Send the goal and wait for the result, with retry logic
            retries = 0
            while retries < max_retries:
                self.client.send_goal(self.move_base_goal, done_cb=self.done_callback)
                self.client.wait_for_result()
                if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    bar.title(f"Reached at ({x}m, {y}m) at {theta} radians")
                    break
                else:
                    bar.text(f"Navigation attempt {retries + 1}/{max_retries} failed. Retrying...")
                    retries += 1
            return

    # x, y, and z are in meters. Uses FunMap clicked_point topic OR MoveArm service
    def pick_up_at_xyz(self, x, y, z, using_service=True):
        with alive_bar(**self.bar_config) as bar:
            if using_service:  # MoveArm service call: Default
                bar.text(f"Sending the pickup point {x}, {y}, {z} to MoveArm service...")
                rospy.wait_for_service("/funmap/move_arm")
                try:
                    move_arm = rospy.ServiceProxy("/funmap/move_arm", MoveArm)
                    self.pick_up_point.x = x
                    self.pick_up_point.y = y
                    self.pick_up_point.z = z
                    response = move_arm(self.pick_up_point)  # Try alternate service call method if it doesn't work
                    bar.title(f"Point {x}, {y}, {z} reached.")
                    return response.message
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                    return None

            else:  # FunMap clicked_point topic: Alternative
                bar.text(f"Sending the pickup point {x}, {y}, {z} to FunMap clicked_point topic...")
                pub = rospy.Publisher("/clicked_point", PoseStamped, queue_size=1)
                self.clicked_point_goal.header.stamp = rospy.Time.now()
                self.clicked_point_goal.header.frame_id = "map"
                self.clicked_point_goal.pose.position.x = x
                self.clicked_point_goal.pose.position.y = y
                z_offset = -0.1  # Adjust this value as needed
                self.clicked_point_goal.pose.position.z = z + z_offset
                pub.publish(self.clicked_point_goal)
                bar.title(f"Point {x}, {y}, {z} reached.")
                return


# End of class
########################################################################################################################
