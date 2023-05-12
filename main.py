#!/usr/bin/env python3

from camera_roboter import CameraRoboter
from math import pi
import rospy

# Hauptprogramm
if __name__ == '__main__':

    CameraRoboterI = CameraRoboter()
    try:

        print("")
        print("Initialize robot.")
        print("")

        CameraRoboterI.return_to_ready_pose()
        CameraRoboterI.update_pose()

        print("Robot ready to move.")
        print("")

        CameraRoboterI.move(CameraRoboterI.x_0, CameraRoboterI.y_0, CameraRoboterI.z_0, CameraRoboterI.roll_angle_0, CameraRoboterI.pitch_angle_0, CameraRoboterI.yaw_angel_0)
        CameraRoboterI.update_pose()

        CameraRoboterI.move_lin_trajectory(0.03, 4, 0)
        CameraRoboterI.update_pose()

        CameraRoboterI.move(CameraRoboterI.x_0, CameraRoboterI.y_0, CameraRoboterI.z_0, CameraRoboterI.roll_angle_0, CameraRoboterI.pitch_angle_0, CameraRoboterI.yaw_angel_0)
        CameraRoboterI.update_pose()

        CameraRoboterI.move_with_angle(pi/25)
        CameraRoboterI.update_pose()

        CameraRoboterI.move_lin_trajectory(0.03, 4, 0)
        CameraRoboterI.update_pose()

        CameraRoboterI.move(CameraRoboterI.x_0, CameraRoboterI.y_0, CameraRoboterI.z_0, CameraRoboterI.roll_angle_0, CameraRoboterI.pitch_angle_0, CameraRoboterI.yaw_angel_0)
        CameraRoboterI.update_pose()

        CameraRoboterI.move_with_angle(-pi/25)
        CameraRoboterI.update_pose()

        CameraRoboterI.move_lin_trajectory(0.03, 4, 0)
        CameraRoboterI.update_pose()

        print("Robot back to the initial position.")
        print("")

        CameraRoboterI.return_to_ready_pose()
        CameraRoboterI.update_pose()

    except rospy.ROSInterruptException:
        pass
