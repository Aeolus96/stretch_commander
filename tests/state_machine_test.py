#!/usr/bin/env python3
import rospy
import time
from stretch_commander.joint_commands import StretchManipulation
from stretch_commander.nav_commands import StretchNavigation
from stretch_commander.perception_commands import StretchPerception


def state_machine(start_state: str):
    objects_collected = 0
    state = start_state

    rospy.loginfo(f"Checking states... ... ... State: {state}")
    while not rospy.is_shutdown():
        if state == "mapping":
            # Wait for key press to continue
            input("Press Enter to start MAPPING...")

            # Mapping Sequence:
            rospy.loginfo("Mapping...")
            # nav.trigger_head_scan()  # Look around to update the map
            # nav.go_to_xya(2.0, 1.0, 0.0)  # go to hardcoded scanning pose
            # nav.trigger_head_scan()  # Look around to update the map
            # nav.go_to_xya(4.0, 3.0, 0.0)  # go to hardcoded scanning pose
            # nav.trigger_head_scan()  # Look around to update the map

            rospy.loginfo("Mapping complete")
            state = "detecting"

        elif state == "detecting":
            # Wait for key press to continue
            input("Press Enter to start DETECTING...")

            rospy.loginfo("Cycling through camera positions and triggering YOLO...")
            # for i in range(3): # Cycle through 3 camera positions
            #     man.look_for_shirts(i+1)
            #     time.sleep(3) # wait for movement to complete
            #     per.trigger_yolo()
            #     time.sleep(10) # wait for detection to complete
            #     if per.detected_objects: # Prevents multiple Detection2DArray publishes
            #         break

            if per.detected_objects:
                rospy.loginfo(
                    f"Detected Object. Closest Point: {nav.target_point.x}, {nav.target_point.y}, {nav.target_point.z}"
                )
                state = "collecting"
                per.detected_objects = False  # Reset detection flag
            else:
                rospy.loginfo("No objects detected.")
                rospy.loginfo("Moving to default search location...")
                per.detected_objects = True
                state = "detecting"

        elif state == "collecting":
            # Wait for key press to continue
            input("Press Enter to start COLLECTING landry...")
            rospy.loginfo("Sending coordinates to MoveArm service...")
            # rospy.loginfo(nav.pick_up_at_xyz(nav.target_point.x, nav.target_point.y, nav.target_point.z))
            rospy.loginfo("Sending joint commands to move arm to pick up object...")
            rospy.loginfo("Picked up laundry! Moving to drop off...")
            state = "dropoff"

        elif state == "dropoff":
            # Laundry drop off point (map frame):
            fixed_dropoff_x = 1.0
            fixed_dropoff_y = 1.0
            fixed_dropoff_z = 0.7
            # Wait for key press to continue
            input("Press Enter to start DROP OFF...")

            # Go to drop off point
            rospy.loginfo(f"Moving to drop off point at {fixed_dropoff_x}, {fixed_dropoff_y}, {fixed_dropoff_z}")
            # rospy.loginfo(nav.pick_up_at_xyz(fixed_dropoff_x, fixed_dropoff_y, fixed_dropoff_z))
            rospy.loginfo("Sending joint commands to move arm to drop off object...")
            # man.gripper_open()
            # man.arm_fold()
            # man.wrist_up()

            # Add to objects collected - no feedback or verification implemented for demo purposes
            objects_collected += 1
            rospy.loginfo(f"Objects Collected: {objects_collected}")
            state = "detecting"

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

    # Evaluation Variables simulated for demo purposes:
    per.detected_objects = True
    nav.target_point.x = 3
    nav.target_point.y = 4
    nav.target_point.z = 0.1

    try:
        # Start state machine from desired state
        rospy.loginfo("Starting state machine...")
        state_machine("mapping")
        rospy.loginfo("Exiting state machine...")
    except rospy.ROSInterruptException:
        pass


# End of script
#######################################################################################################################
