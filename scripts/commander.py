#!/usr/bin/env python3
import rospy

from stretch_commander.joint_commands import StretchManipulation
from stretch_commander.nav_commands import StretchNavigation
from stretch_commander.perception_commands import StretchPerception

# import stretch_commander.nav_commands import StretchNavigation

if __name__ == "__main__":
    rospy.init_node("stretch_commander")
    nav = StretchNavigation()
    man = StretchManipulation()
    per = StretchPerception()
    
    # man.trigger_home_the_robot()
    nav.trigger_global_localization()
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

    per.find_clothes() # gives a list of clothes
