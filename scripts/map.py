#!/usr/bin/env python3
import rospy
from stretch_commander.joint_commands import StretchManipulation
from stretch_commander.nav_commands import StretchNavigation
from stretch_commander.perception_commands import StretchPerception

if __name__ == "__main__":
    rospy.init_node("stretch_commander")

    # Initialize the modules:
    nav = StretchNavigation()
    man = StretchManipulation()
    per = StretchPerception()

    # Wait for key press to continue
    input("Press Enter to start mapping...")

    # Mapping Sequence:
    nav.trigger_head_scan()  # Look around to update the map
    nav.go_to_xya(2.0, 1.0, 0.0)  # go to hardcoded scanning pose
    nav.trigger_head_scan()  # Look around to update the map
    nav.go_to_xya(4.0, 3.0, 0.0)  # go to hardcoded scanning pose
    nav.trigger_head_scan()  # Look around to update the map
    
    print("Mapping complete")