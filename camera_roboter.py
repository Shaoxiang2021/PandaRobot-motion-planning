#!/usr/bin/env python3
#Importiere Bibliotheken

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sin, cos
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

# from geometry_msgs.msg import Twist

# from ueye_camera import UeyeCamera

class CameraRoboter():
    def __init__(self):

        # Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('IRP_Python', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        # einstelle Toleranz
        self.group.set_goal_joint_tolerance(0.0001)
        self.group.set_goal_position_tolerance(0.0001)
        self.group.set_goal_orientation_tolerance(0.001)

        self.end_effector_link = self.group.get_end_effector_link()
        self.reference_frame = 'panda_link0'
        self.group.set_pose_reference_frame(self.reference_frame)

        # self.group.set_planning_time(5)
        # self.group.allow_replanning(True)
        # self.group.set_planner_id("RRTstar")

        self.group.set_max_acceleration_scaling_factor(1)
        self.group.set_max_velocity_scaling_factor(1)

        self.set_scene()

        # Initialisiere Anfangspose, die nach dem Kalibrierensvorgang korriegiert wird
        self.x_init = 0.4 # Arbeitsdistanz ungefähr um 0.35 liegen
        self.y_init = 0
        self.z_init = 0.4
        self.roll_angle_init = pi
        self.pitch_angle_init = 0
        self.yaw_angel_init = pi/4

        # Initialisiere Position von Roboter
        self.pose = self.group.get_current_pose().pose


        # Initailisiere Kamera
        # self.Camera = UeyeCamera()

        # Initialisiere relevante Parameter für Kamera
        # self.exposure_time = 10

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
    
    def set_scene(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        # rospy.sleep(1)
        table_id_1 = 'table'
        # self.scene.remove_world_object(table_id_1)
        # rospy.sleep(1)
        table_id_2 = 'workspace_left'
        table_id_3 = 'workspace_right'
        table_id_4 = 'workspace_front'
        table_id_5 = 'workspace_back'
        table_size_1 = [1, 1, 0.01]
        table_pose_1 = geometry_msgs.msg.PoseStamped()
        table_pose_1.header.frame_id = self.reference_frame
        table_pose_1.pose.position.x = 0.8
        table_pose_1.pose.position.y = 0.0
        table_pose_1.pose.position.z = 0.3
        table_pose_1.pose.orientation.w = 1.0
        self.scene.add_box(table_id_1, table_pose_1, table_size_1)

    def move_pose(self, x, y, z, roll, pitch, yaw):

        # Zielpose festlegen
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        # Zielpose zum Buffer der Bewegungsplanung hinzufügen
        self.group.set_pose_target(pose_goal)

        # Führe die Bewegung aus
        self.group.go(wait=True)

        # Stoppe den Roboter
        self.group.stop()

        # Löschen die Zielposition
        self.group.clear_pose_targets()
    
    def move_position(self, x, y, z):

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

        # Stoppe den Roboter
        self.group.stop()

        # Löschen die Zielposition
        self.group.clear_pose_targets()

    def caculation_pose(self, angle):
        return [self.pose.position.y + self.pose.position.z*sin(angle), self.pose.position.z*cos(angle)]

    def change_angle(self, angle):

        # Berechne Euler Winkel des Roboters
        quaternion = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        euler_anlge = euler_from_quaternion(quaternion)

        # Umrechne Zielpose
        roll = euler_anlge[0] - angle
        # pitch = euler_anlge[1]
        # yaw = euler_anlge[2]
        pitch = 0
        yaw = -pi/4
        x = self.pose.position.x
        y, z = self.caculation_pose(angle)

        # Führe die Bewegung
        self.move_pose(x, y, z, roll, pitch, yaw)

    def move_linear(self, x, y, z):

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

    def rotate(self, angle):

        # Lese die aktuelle Pose des Roboters aus und beeschreibe die Joint-Ziel-Pose
        joint_goal = self.group.get_current_joint_values()
        joint_goal[6] = joint_goal[6] + angle

        # Führe die Bewegung aus
        self.group.go(joint_goal, wait=True)

        # Stoppe den Roboter
        self.group.stop()
 
    def move_trajectory(self, moving_step, num_step, sleep_time):
        for i in range(num_step):
            x = self.pose.position.x + (i+1)*moving_step
            self.move_position(x, self.pose.position.y, self.pose.position.z)
            rospy.sleep(sleep_time)

    def move_linear_trajectory(self, moving_step, num_step, sleep_time, take_image=False):
        self.get_image(take_image)
        for i in range(num_step):
            x = self.pose.position.x + (i+1)*moving_step
            self.move_linear(x, self.pose.position.y, self.pose.position.z)
            rospy.sleep(sleep_time)
            self.get_image(take_image)

    def return_to_ready_pose(self):
        self.move_joint(0, -pi/4, 0, -3*pi/4, 0, pi/2, -pi/4)

    def show_position(self):
        print(self.group.get_current_pose().pose)

    def update_pose(self):
        self.pose = self.group.get_current_pose().pose

    def get_image(self, take_image):
        if take_image is True:
            self.Camera.save_image()

    def get_video_stream(self):
        self.Camera.get_video_stream()

    def calibration(self):
        pass

    def keyboard_control(self):
        pass

    def take_action(self):

        self.return_to_ready_pose()
        self.update_pose()

        self.move_position(self.x_init, self.y_init, self.z_init-0.05)
        self.update_pose()

        self.move_linear_trajectory(0.02, 5, 0.5, take_image=True)
        self.update_pose()

        self.return_to_ready_pose()
        self.update_pose()
