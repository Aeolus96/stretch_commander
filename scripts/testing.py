#!/usr/bin/env python3

import rospy
from stretch_commander.test import say_it_works

if __name__ == '__main__':
    rospy.init_node('test_node')
    say_it_works()
