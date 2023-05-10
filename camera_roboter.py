#!/usr/bin/env python3
#Importiere Bibliotheken

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sin, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler

from ueye_camera import UeyeCamera

class CameraRoboter():
    def __init__(self):

        # Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('IRP_Python', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Toleranzen
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.05)

        # Initailisiere Kamera
        self.Camera = UeyeCamera()

        # Initialisiere relevante Parameter für Roboter
        self.x = 0.4
        self.y = 0
        self.z = 0.25
        self.roll_angle = pi/4
        self.pitch_angle = pi
        self.yaw_angel = 0

        # Initialisiere relevante Parameter für Kamera
        # self.exposure_time = 10

    def caculation_pose(self, angle):
        return [self.z*sin(angle), self.z*cos(angle)]

    def move_robot(self, x, y, z, roll, pitch, yaw):

        # Zielpose festlegen
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.w = quaternion[0]
        pose_goal.orientation.x = quaternion[1]
        pose_goal.orientation.y = quaternion[2]
        pose_goal.orientation.z = quaternion[3]

        # Zielpose zum Buffer der Bewegungsplanung hinzufügen
        self.group.set_pose_target(pose_goal)

        # Führe die Bewegung aus
        self.group.go(wait=True)

        #Stoppe den Roboter
        self.group.stop()

        # Löschen die Zielposition
        self.group.clear_pose_targets()

    def move_lin_robot(self, x, y, z):

        # Aktuelle Pose auslegen und Zielpose festlegen
        home_pose=self.group.get_current_pose().pose
        pose=copy.deepcopy(home_pose)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Plane die Wegpunkte
        waypoints = []
        waypoints.append(copy.deepcopy(pose))
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

        # Führe die Bewegung aus
        self.group.execute(plan, wait=True)

    def move_joint_robot(self, j0, j1, j2, j3, j4, j5, j6):

        # Lese die aktuelle Pose des Roboters aus und beeschreibe die Joint-Ziel-Pose
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = j0
        joint_goal[1] = j1
        joint_goal[2] = j2
        joint_goal[3] = j3
        joint_goal[4] = j4
        joint_goal[5] = j5
        joint_goal[6] = j6

        # Führe die Bewegung aus
        self.group.go(joint_goal, wait=True)

        # Stoppe den Roboter
        self.group.stop()

    def combine_move(self, mode, moving_step, num_step, angle):
        roll = self.roll_angle
        pitch = self.pitch_angle
        yaw = self.yaw_angel - angle
        y, z = self.caculation_pose(angle)
        for i in range(num_step):
            if mode[i] == 0:
                # j0, j1, j2, j3, j4, j5, j6 = 
                # self.move_joint_robot(j0, j1, j2, j3, j4, j5, j6)
                pass
            elif mode[i] == 1:
                x = self.x + i*moving_step
                self.move_robot(x, y, z, roll, pitch, yaw)
            elif mode[i] == 2:
                x = self.x + i*moving_step
                self.move_lin_robot(x, y, z)
            else:
                print("Error input, mode should be 0-2!")


    def return_to_ready_pose(self):
        self.move_joint_robot(0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4)

    def show_position(self):
        print(self.group.get_current_pose().pose)

    def get_image(self):
        self.Camera.save_image()
