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
roslaunch stretch_funmap mapping.launch map_yaml:=/home/hello-robot/stretch_user/debug/merged_maps/merged_map_20231030190502 rviz:=false
"""
# <---

# Calibrate the robot motors (every time robots boots up):
"""
rosservice call /calibrate_the_robot "{}"
"""

# Start of script:
#######################################################################################################################
if __name__ == "__main__":
    rospy.init_node("stretch_commander")

    # Initialize the modules:
    nav = StretchNavigation()
    man = StretchManipulation()
    per = StretchPerception()

    # Wait for key press to continue
    input("Press Enter to continue...")

    max_retries = 3  # Limit the number of searches
    retries = 0
    while retries < max_retries:
        # Perception:
        # TODO: Look around again to detect the objects (Manipulation Camera Joints)

        if not per.detected_objects:  # No objects detected -> go to nearby spot
            if retries == 0:
                nav.go_to_xya(2.0, 1.0, 0.0)  # Hardcoded location for now
            elif retries == 1:
                nav.go_to_xya(4.0, 3.0, 0.0)  # Hardcoded location for now
            elif retries == 2:
                nav.go_to_xya(5.0, 3.0, 0.0)  # Hardcoded location for now

        else:  # Object detected -> go to object
            # TODO: Bounding Boxes Array (Detection2D Array from YOLO node) -> 2D Pose Array
            # TODO: 2D Pose Array -> Closest Point or some selection method

            # Manipulation:
            # TODO: Closest Point -> FUNMAP (Navigation using MoveArm)
            # TODO: Pickup Object

            x = man.targetPoint.x
            y = man.targetPoint.y
            z = man.targetPoint.z
            
            nav.pick_up_at_xyz(x, y, z)
            

            # Navigation:
            # TODO: Go to drop off point (Perception: IF time allows, moving drop point)
            pass

        retries += 1

# End of script
#######################################################################################################################
