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

from ueye_camera import UeyeCamera
from data_manager import DataManager

class CameraRoboter():
    def __init__(self, load_camera=False):

        # Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('IRP_Python', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        # self.robot = moveit_commander.MoveGroupCommander('manipulator')
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

        # Initailisiere Kamera und DataManager
        if load_camera is True:
            self.Camera = UeyeCamera()
        self.DataManager = DataManager()

        self.sleep_time = 0.5

        # self.set_scene()

        # Initialisiere Anfangspose, die nach dem Kalibrierensvorgang korriegiert wird
        self.x_init = 0.43
        self.y_init = 0
        self.z_init = 0.35
        self.roll_angle_init = pi
        self.pitch_angle_init = 0
        self.yaw_angel_init = pi/4

        self.brightness_class = list()

        # Initialisiere Position von Roboter
        # self.pose = self.group.get_current_pose().pose
        self.return_to_ready_pose()

        # Initialisiere relevante Parameter für Kamera
        # self.exposure_time = 10

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        self.DataManager.print_and_write_into_log("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        self.DataManager.print_and_write_into_log("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        self.DataManager.print_and_write_into_log("============ Available Planning Groups: %s" % group_names)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
    
    def set_scene(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.wait()
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
        table_pose_1.pose.position.z = 0.1
        table_pose_1.pose.orientation.w = 1.0
        self.scene.add_box(table_id_1, table_pose_1, table_size_1)

        self.DataManager.print_and_write_into_log("Initialize workspace...")
    
    def set_brightness_class(self, brightness_class):
        self.brightness_class = brightness_class

    def move_pose(self, x, y, z, roll, pitch, yaw, image_index=0, save_image=False):

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

        # Aufnahme ein Bild
        self.wait()
        self.get_image(save_image, image_index)
    
    def move_position(self, x, y, z, image_index=0, save_image=False):

        pose = self.group.get_current_pose().pose

        # Zielpose festlegen
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        pose_goal.orientation.w = pose.orientation.w
        pose_goal.orientation.x = pose.orientation.x
        pose_goal.orientation.y = pose.orientation.y
        pose_goal.orientation.z = pose.orientation.z

        # Zielpose zum Buffer der Bewegungsplanung hinzufügen
        self.group.set_pose_target(pose_goal)

        # Führe die Bewegung aus
        self.group.go(wait=True)

        # Stoppe den Roboter
        self.group.stop()

        # Löschen die Zielposition
        self.group.clear_pose_targets()

        # Aufnahme ein Bild
        self.wait()
        self.get_image(save_image, image_index)

    '''
    def caculation_pose(self, angle):
        pose = self.group.get_current_pose().pose
        return [pose.position.y + pose.position.z*sin(angle), pose.position.z*cos(angle)]
    '''

    def move_linear(self, x, y, z, image_index=0, save_image=False):

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

        # Aufnahme ein Bild
        self.wait()
        self.get_image(save_image, image_index)

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

    def change_angle(self, angle):

        pose = self.group.get_current_pose().pose

        # Berechne Euler Winkel des Roboters
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler_anlge = euler_from_quaternion(quaternion)

        # Umrechne Zielpose
        roll = euler_anlge[0] - angle
        pitch = 0
        yaw = 0
        x = pose.position.x
        y = pose.position.y + pose.position.z*sin(angle)
        z = pose.position.z*cos(angle)

        # Führe die Bewegung
        self.move_pose(x, y, z, roll, pitch, yaw)

        self.rotate(-pi/4)
    
    '''
    def move_trajectory(self, moving_step, num_step, save_image=False):
        pose = self.group.get_current_pose().pose
        for i in range(num_step):
            x = pose.position.x + (i+1)*moving_step
            self.move_position(x, pose.position.y, pose.position.z, save_image=save_image)
    '''

    def move_linear_trajectory(self, moving_step, num_step, save_image=False):
        pose = self.group.get_current_pose().pose
        for i in range(num_step):
            x = pose.position.x + (i+1)*moving_step
            self.move_linear(x, pose.position.y, pose.position.z, image_index=i+1, save_image=save_image)

    def return_to_ready_pose(self):
        self.move_joint(0, -pi/4, 0, -3*pi/4, 0, pi/2, -pi/4)

    def show_position(self):
        print(self.group.get_current_pose().pose)

    # def update_pose(self):
        # self.pose = self.group.get_current_pose().pose

    def get_image(self, take_image, image_index):
        pose = self.group.get_current_pose().pose
        if take_image is True:
            filename = self.DataManager.generate_image_path(pose.position.x, image_index)
            self.Camera.save_image(filename)

    def show_image(self):
        self.Camera.show_image()

    def get_video_stream(self):
        self.Camera.get_video_stream()

    def close_camera(self):
        self.Camera.close()

    def calibration(self):
        self.move_position(self.x_init, self.y_init, self.z_init)
        self.show_image()

    def keyboard_control(self):
        pass

    def wait(self):
        rospy.sleep(self.sleep_time)

    def take_action(self, save_image=False):
        
        try:
            '''
            self.calibration()
            self.return_to_ready_pose()

            self.DataManager.ask_example_info()

            for i in range(len(self.brightness_class)):
                self.DataManager.set_brightness(self.brightness_class[i])
                self.move_position(self.x_init, self.y_init, self.z_init, save_image=save_image)
                self.move_linear_trajectory(0.016, 6, save_image=save_image)
                self.return_to_ready_pose()
                input("press any key to continue...")
            '''
            pass

        finally:
            self.close_camera()
