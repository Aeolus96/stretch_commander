#!/usr/bin/env python3
import rospy
import time
from stretch_commander.joint_commands import StretchManipulation
from stretch_commander.nav_commands import StretchNavigation
from stretch_commander.perception_commands import StretchPerception
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray, Detection2D, Detection2DArray

# Commands to start everything on the robot (using ssh on robot):
"""
roslaunch stretch_funmap mapping.launch rviz:=false
roslaunch synchronized_throttle synchronize_stretch.launch
roslaunch synchronized_throttle generate_pcloud.launch
"""
# Use the custom RViz config (lightweight and fast):
"""
rviz -d $(rospack find stretch_commander)/rviz/command_view.rviz
"""
# DEPRECATED --->
"""
roslaunch stretch_funmap mapping.launch map_yaml:=/home/hello-robot/stretch_user/debug/merged_maps/merged_map_20231130152931 rviz:=false
"""
# <---

# Calibrate the robot motors (every time robots boots up):
"""
rosservice call /calibrate_the_robot "{}"
"""


def state_machine(start_state: str):
    objects_collected = 0
    state = start_state

    while not rospy.is_shutdown():
        if state == "mapping":
            # Wait for key press to continue
            input("Press Enter to start mapping...")

            # Mapping Sequence:
            nav.trigger_head_scan()  # Look around to update the map
            nav.go_to_xya(2.0, 1.0, 0.0)  # go to hardcoded scanning pose
            nav.trigger_head_scan()  # Look around to update the map
            nav.go_to_xya(4.0, 3.0, 0.0)  # go to hardcoded scanning pose
            nav.trigger_head_scan()  # Look around to update the map

            rospy.loginfo("Mapping complete")
            state = "detecting"

        elif state == "detecting":
            # Wait for key press to continue
            
            input("Press Enter to start detecting...")

            for i in range(3):  # Cycle through 3 camera positions
                man.look_for_shirts(i + 1)
                time.sleep(3)  # wait for movement to complete
                per.trigger_yolo()
                #per.publish_test_box()
                time.sleep(10)  # wait for detection to complete
                if per.detected_objects:  # Prevents multiple Detection2DArray publishes
                    break

            if per.detected_objects:
                rospy.loginfo(
                    f"Detected Object. Closest Point: {nav.target_point.x}, {nav.target_point.y}, {nav.target_point.z}"
                )
                state = "collecting"
                man.arm_up()
                per.detected_objects = False  # Reset detection flag
            else:
                rospy.loginfo("No objects detected")
                state = "detecting"

        elif state == "collecting":
            # Get ready to pick up:
            man.arm_up()
            man.gripper_open()
            man.wrist_down()
            time.sleep(1)
            # Wait for key press to continue
            input("Press Enter to start picking up landry...")
            rospy.loginfo(nav.pick_up_at_xyz(nav.target_point.x, nav.target_point.y, 1.0))
            man.arm_extend()
            man.arm_down()
            time.sleep(5)
            man.gripper_close()
            time.sleep(1)
            man.arm_up()
            # man.arm_fold() # Not needed for demo purposes, might obstruct the lidar
            state = "dropoff"

        elif state == "dropoff":
            # Laundry drop off point (map frame):
            fixed_dropoff_x = 1.492
            fixed_dropoff_y = -0.274
            fixed_dropoff_z = 1.0
            # Wait for key press to continue
            input("Press Enter to start drop off...")

            # Go to drop off point
            rospy.loginfo(nav.pick_up_at_xyz(fixed_dropoff_x, fixed_dropoff_y, fixed_dropoff_z))
            man.gripper_open()
            man.wrist_up()
            man.wrist_in()
            man.arm_fold()

            # Add to objects collected - no feedback or verification implemented for demo purposes
            objects_collected += 1

            # Exit Condition:
            if objects_collected >= 2:  # 2 objects available for demo purposes
                rospy.loginfo("All objects collected! Exiting...")
                return

            state = "detecting"


# Start of script:
#######################################################################################################################
if __name__ == "__main__":
    rospy.init_node("stretch_commander")

    # Initialize the modules:
    nav = StretchNavigation()
    man = StretchManipulation()
    per = StretchPerception()

    try:
        # Start state machine from desired state, mapping | detecting | collecting | dropoff
        state_machine("detecting")  # starting with detecting for testing purposes
    except rospy.ROSInterruptException:
        pass

# End of script
#######################################################################################################################
