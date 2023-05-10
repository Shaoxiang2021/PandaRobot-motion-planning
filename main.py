#!/usr/bin/env python3

from camera_roboter import CameraRoboter
from math import pi
import rospy

# Hauptprogramm
if __name__ == '__main__':

    CameraRoboterI = CameraRoboter()
    try:
        mode = [1, 2, 2, 2, 2, 2]
        moving_step = 0.03
        num_step = 4
        angle = -pi/45

        print("")
        print("Initialize robot.")
        print("")

        CameraRoboterI.return_to_ready_pose()

        print("Robot ready to move.")
        print("")

        CameraRoboterI.combine_move(mode, moving_step, num_step, angle)

        print("Robot back to the initial position.")
        print("")

        CameraRoboterI.return_to_ready_pose()

    except rospy.ROSInterruptException:
        pass
