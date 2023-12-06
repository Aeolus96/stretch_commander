#!/usr/bin/env python3
import rospy
from stretch_commander.joint_commands import StretchManipulation
from stretch_commander.nav_commands import StretchNavigation
from stretch_commander.perception_commands import StretchPerception

# Commands to start everything on the robot (using ssh on robot):
"""
roslaunch stretch_funmap mapping.launch rviz:=false
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

            # Perception:
            # TODO: Look around again to detect the objects (Manipulation Camera Joints)
            # TODO: Change Camera Joints to look around and Trigger Yolo node
            # TODO: Get Bounding Boxes Array (Detection2D Array from YOLO node) -> Closest Point for Pickup

            man.look_for_shirts(1)
            per.trigger_yolo()
            man.look_for_shirts(2)
            per.trigger_yolo()
            man.look_for_shirts(3)
            per.trigger_yolo()

            if per.detected_objects:
                rospy.loginfo(f"Detection complete. Closest Point: {0.0}, {0.0}, {0.0}")
                state = "collecting"
                per.detected_objects = False
            else:
                rospy.loginfo("No objects detected")
                state = "detecting"

        elif state == "collecting":
            # Wait for key press to continue
            input("Press Enter to start picking up landry...")

            # Manipulation:
            # TODO: Closest Point -> FUNMAP (Navigate using MoveArm)
            # TODO: Pickup Object (using Joint Commands)

        elif state == "dropoff":
            # Laundry drop off point (map frame):
            fixed_dropoff_x = 0.0
            fixed_dropoff_y = 0.0
            fixed_dropoff_z = 0.7
            # Wait for key press to continue
            input("Press Enter to start drop off...")

            # Go to drop off point
            nav.pick_up_at_xyz(fixed_dropoff_x, fixed_dropoff_y, fixed_dropoff_z)
            man.gripper_open()
            man.arm_fold()
            man.wrist_up()

            # Add to objects collected - no feedback or verification implemented for demo purposes
            objects_collected += 1

            # Exit Condition:
            if objects_collected >= 3:
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

    # Start state machine from desired state
    state_machine("mapping")


# End of script
#######################################################################################################################
