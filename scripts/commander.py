#!/usr/bin/env python3
import rospy
from stretch_commander.joint_commands import StretchManipulation
from stretch_commander.nav_commands import StretchNavigation
from stretch_commander.perception_commands import StretchPerception
# from alive_progress import alive_bar

# Commands to start everything on the robot (using ssh on robot):
"""
roslaunch stretch_funmap mapping.launch map_yaml:=/home/hello-robot/stretch_user/debug/merged_maps/merged_map_20231030190502 rviz:=false
"""
# Calibrate the robot motors (every time robots boots up):
"""
rosservice call /calibrate_the_robot "{}"
"""


# Old Testing Commands: deprecated nav commands
"""
    man.arm_up()
    man.arm_fold()
    nav.go_to(4.3, 2.8, 2.0)
    man.wrist_out()
    man.wrist_down()
    man.arm_extend()
    man.gripper_open()
    rospy.sleep(2.0)
    man.arm_down()
    man.gripper_close()
    rospy.sleep(2.0)
    man.wrist_up()
    man.arm_up()
    man.wrist_in()
    rospy.sleep(2.0)
    man.arm_fold()
    nav.go_to(5.30, 5.75, 0.0)
    man.wrist_out()
    man.gripper_open()
"""
# TODO: ^^^these manipulation commands can be wrapped up into 2-3 lists or final positions like stowed/ready/reach


if __name__ == "__main__":
    rospy.init_node("stretch_commander")

    # Initialize the modules:
    nav = StretchNavigation()
    man = StretchManipulation()
    per = StretchPerception()

    # Navigation Initialization:
    nav.trigger_global_localization()  # locate the robot on the map:
    nav.go_to_xya(4.3, 2.8, 0.0)  # go to hardcoded start pose
    nav.trigger_head_scan()  # Look around to update the map
    # Wait for key press to continue
    input("Press Enter to continue...")

    max_retries = 3  # Limit the number of searches
    retries = 0
    while retries < max_retries:
        # TODO: Check if nested alive bars work independently in separate lines

        # Perception:
        # TODO: Look around again to detect the objects (Manipulation Camera Joints)

        if not per.detected_objects:  # No objects detected -> go to nearby spot
            if retries == 0:
                nav.go_to_xya(5.3, 2.8, 0.0)  # Hardcoded location for now
            elif retries == 1:
                nav.go_to_xya(4.0, 3.0, 0.0)  # Hardcoded location for now
            elif retries == 2:
                nav.go_to_xya(4.5, 3.5, 0.0)  # Hardcoded location for now

        else:  # Object detected -> go to object
            # TODO: Bounding Boxes Array (Detection2D Array from YOLO node) -> 2D Pose Array
            # TODO: 2D Pose Array -> Closest Point or some selection method

            # Manipulation:
            # TODO: Closest Point -> FUNMAP (Navigation using MoveArm)
            # TODO: Pickup Object

            # Navigation:
            # TODO: Go to drop off point (Perception: IF time allows, moving drop point)
            pass

        retries += 1
