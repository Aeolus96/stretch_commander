import sys  # for sys.exit

import actionlib  # for MoveBase
import rospy  # for ROS Python
from geometry_msgs.msg import Quaternion  # for MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction  # for MoveBase client
from move_base_msgs.msg import MoveBaseGoal
from std_srvs.srv import Trigger  # for Triggering services
from tf import transformations  # for quaternion_from_euler


########################################################################################################################
class StretchNavigation:
    def __init__(self, frame_id="map"):
        # Set up the MoveBase client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        if not self.client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Failed to connect to the move_base server.")
            sys.exit(1)  # exit because Navigation wont work without a MoveBase server

        rospy.loginfo(f"<CHECK> {self.__class__.__name__}: Made contact with move_base server")
        # define the goal message
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = frame_id

    # Re-locates the robot in the loaded map
    def trigger_global_localization(self):
        rospy.wait_for_service("/funmap/trigger_global_localization")
        try:
            trigger_loc = rospy.ServiceProxy("/funmap/trigger_global_localization", Trigger)
            response = trigger_loc()
            return response.message
        except rospy.ServiceException as e:
            rospy.logerr(f"Global localization service call failed: {e}")
            return None

    # Scans the surrounding area using the RealSense depth camera
    def trigger_head_scan(self):
        rospy.wait_for_service("/funmap/trigger_head_scan")
        try:
            trigger_scan = rospy.ServiceProxy("/funmap/trigger_head_scan", Trigger)
            response = trigger_scan()
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

    # x, y, and theta are in meters and radians. Use math.radians(theta) to use degrees
    def go_to(self, x, y, theta, max_retries=3):
        if not isinstance(max_retries, int) or max_retries <= 0:
            raise ValueError("max_retries should be a positive integer.")
        # Other error checking not needed for this application (though could be added)
        
        rospy.loginfo(f"{self.__class__.__name__}: Heading for ({x}, {y}) at {theta} radians")

        self.goal.target_pose.header.stamp = rospy.Time()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation = self.get_quaternion(theta)

        # Send the goal and wait for the result, with retry logic
        retries = 0
        while retries < max_retries:
            self.client.send_goal(self.goal, done_cb=self.done_callback)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                break
            else:
                rospy.loginfo(f"<CHECK> {self.__class__.__name__}: Goal attempt {retries + 1}/{max_retries} failed.")
                retries += 1


# End of class
########################################################################################################################
