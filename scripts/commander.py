#!/usr/bin/env python3

from stretch_commander import (StretchManipulation, StretchNavigation,
                               StretchPerception)

# import stretch_commander.nav_commands import StretchNavigation

if __name__ == "__main__":
    rospy.init_node("stretch_commander")
    nav = StretchNavigation()
    man = StretchManipulation()
    per = StretchPerception()
    
    man.lift_up()
    man.gripper_open()
    man.reach_down()
    # nav.trigger_global_localization()
    # nav.go_to(4.3, 2.8, 0.0)
