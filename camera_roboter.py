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
from tf.transformations import euler_from_quaternion

import time

# from ueye_camera import UeyeCamera

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
        self.group.set_goal_position_tolerance(0.00001)
        self.group.set_goal_orientation_tolerance(0.00001)

        # Initailisiere Kamera
        # self.Camera = UeyeCamera()

        # Initialisiere Anfangspose, die nach dem Kalibrierensvorgang korriegiert wird
        self.x_0 = 0.4
        self.y_0 = 0
        self.z_0 = 0.4
        self.roll_angle_0 = pi/4
        self.pitch_angle_0 = pi
        self.yaw_angel_0 = 0

        # Initialisiere Position von Roboter
        self.pose = self.group.get_current_pose().pose

        # Initialisiere relevante Parameter für Kamera
        # self.exposure_time = 10

    def caculation_pose(self, angle):
        return [self.pose.position.y + self.pose.position.z*sin(angle), self.pose.position.z*cos(angle)]

    def move(self, x, y, z, roll, pitch, yaw):

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

        # Stoppe den Roboter
        self.group.stop()

        # Löschen die Zielposition
        self.group.clear_pose_targets()
    
    def move_keep_orientation(self, x, y, z):

        # Zielpose festlegen
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        pose_goal.orientation.w = self.pose.orientation.w
        pose_goal.orientation.x = self.pose.orientation.x
        pose_goal.orientation.y = self.pose.orientation.y
        pose_goal.orientation.z = self.pose.orientation.z

        # Zielpose zum Buffer der Bewegungsplanung hinzufügen
        self.group.set_pose_target(pose_goal)

        # Führe die Bewegung aus
        self.group.go(wait=True)

        #Stoppe den Roboter
        self.group.stop()

        # Löschen die Zielposition
        self.group.clear_pose_targets()

    def move_with_angle(self, angle):
        quaternion = [self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z]
        euler_anlge = euler_from_quaternion(quaternion)
        roll = euler_anlge[0]
        pitch = euler_anlge[1]
        yaw = euler_anlge[2] - angle
        x = self.pose.position.x
        y, z = self.caculation_pose(angle)
        self.move(x, y, z, roll, pitch, yaw)

    def move_lin(self, x, y, z):

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

    def move_joint(self, j0, j1, j2, j3, j4, j5, j6):

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

    def move_trajectory(self, moving_step, num_step):
        for i in range(num_step):
            x = self.pose.position.x + (i+1)*moving_step
            self.move_keep_orientation(x, self.pose.position.y, self.pose.position.z)

    def move_lin_trajectory(self, moving_step, num_step, sleep_time):
        for i in range(num_step):
            x = self.pose.position.x + (i+1)*moving_step
            self.move_lin(x, self.pose.position.y, self.pose.position.z)
            time.sleep(sleep_time)

    def return_to_ready_pose(self):
        self.move_joint(0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4)

    def show_position(self):
        print(self.group.get_current_pose().pose)

    def update_pose(self):
        self.pose = self.group.get_current_pose().pose

    def get_image(self):
        self.Camera.save_image()
