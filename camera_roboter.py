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
        self.group.set_goal_joint_tolerance(0.00001)
        self.group.set_goal_position_tolerance(0.00001)
        self.group.set_goal_orientation_tolerance(0.0001)

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
            self.Camera = UeyeCamera(exposure_ms=7)
        self.DataManager = DataManager()

        self.sleep_time = 0.5

        self.set_scene()

        # Initialisiere Anfangspose, die nach dem Kalibrierensvorgang korriegiert wird
        self.x_init = 0.49
        self.y_init = 0
        self.z_init = 0.35
        self.roll_angle_init = pi
        self.pitch_angle_init = 0
        self.yaw_angel_init = pi/4

        self.joints_ready = [-0.0005271619215060288, -0.06608364050639302, 0.0006570654998020583, -2.372600785872206, 0.0006102935707507034, 2.313072631678507, -0.7852934202783396]

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
        table_id_2 = 'light_left'
        table_id_3 = 'light_right'
        table_id_4 = 'computer'
        table_id_5 = 'work_left'
        table_id_6 = 'work_right'

        table_size_1 = [1, 0.8, 0.01]
        table_pose_1 = geometry_msgs.msg.PoseStamped()
        table_pose_1.header.frame_id = self.reference_frame
        table_pose_1.pose.position.x = 0.65
        table_pose_1.pose.position.y = 0.0
        table_pose_1.pose.position.z = 0.2
        table_pose_1.pose.orientation.w = 1.0
        self.scene.add_box(table_id_1, table_pose_1, table_size_1)

        table_size_2 = [0.35, 0.1, 0.3]
        table_pose_2 = geometry_msgs.msg.PoseStamped()
        table_pose_2.header.frame_id = self.reference_frame
        table_pose_2.pose.position.x = 0.48
        table_pose_2.pose.position.y = -0.14
        table_pose_2.pose.position.z = 0.15
        table_pose_2.pose.orientation.w = 1.0
        self.scene.add_box(table_id_2, table_pose_2, table_size_2)

        table_size_3 = [0.35, 0.1, 0.3]
        table_pose_3 = geometry_msgs.msg.PoseStamped()
        table_pose_3.header.frame_id = self.reference_frame
        table_pose_3.pose.position.x = 0.48
        table_pose_3.pose.position.y = 0.14
        table_pose_3.pose.position.z = 0.15
        table_pose_3.pose.orientation.w = 1.0
        self.scene.add_box(table_id_3, table_pose_3, table_size_3)

        table_size_4 = [0.23, 0.6, 0.55]
        table_pose_4 = geometry_msgs.msg.PoseStamped()
        table_pose_4.header.frame_id = self.reference_frame
        table_pose_4.pose.position.x = 0.18
        table_pose_4.pose.position.y = -0.60
        table_pose_4.pose.position.z = 0.2775
        table_pose_4.pose.orientation.w = 1.0
        self.scene.add_box(table_id_4, table_pose_4, table_size_4)

        '''
        table_size_5 = [0.4, 0.005, 1]
        table_pose_5 = geometry_msgs.msg.PoseStamped()
        table_pose_5.header.frame_id = self.reference_frame
        table_pose_5.pose.position.x = 0
        table_pose_5.pose.position.y = -0.2
        table_pose_5.pose.position.z = 0.5
        table_pose_5.pose.orientation.w = 1.0
        self.scene.add_box(table_id_5, table_pose_5, table_size_5)

        table_size_6 = [0.4, 0.005, 1]
        table_pose_6 = geometry_msgs.msg.PoseStamped()
        table_pose_6.header.frame_id = self.reference_frame
        table_pose_6.pose.position.x = 0
        table_pose_6.pose.position.y = 0.2
        table_pose_6.pose.position.z = 0.5
        table_pose_6.pose.orientation.w = 1.0
        self.scene.add_box(table_id_6, table_pose_6, table_size_6)
        '''

        self.DataManager.print_and_write_into_log("Initialize workspace...")
    
    def set_brightness_class(self, brightness_class):
        self.brightness_class = brightness_class

    def move_pose(self, x, y, z, roll, pitch, yaw, save_image=False):

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
        self.get_image(save_image)
    
    def move_position(self, x, y, z, save_image=False):

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
        self.get_image(save_image)

    '''
    def caculation_pose(self, angle):
        pose = self.group.get_current_pose().pose
        return [pose.position.y + pose.position.z*sin(angle), pose.position.z*cos(angle)]
    '''

    def move_linear(self, x, y, z, save_image=False):

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
        self.get_image(save_image)

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
    
    def move_ready_pose(self, save_image=False):
        [j0, j1, j2, j3, j4, j5, j6] = self.joints_ready
        self.move_joint(j0, j1, j2, j3, j4, j5, j6)

        # Aufnahme ein Bild
        self.wait()
        self.get_image(save_image)

    def rotate(self, angle):

        # Lese die aktuelle Pose des Roboters aus und beeschreibe die Joint-Ziel-Pose
        joint_goal = self.group.get_current_joint_values()
        joint_goal[6] = joint_goal[6] + angle

        # Führe die Bewegung aus
        self.group.go(joint_goal, wait=True)

        # Stoppe den Roboter
        self.group.stop()
    
    def displacement_in_z(self, distance, save_image):

        self.move_position(self.x_init, self.y_init, self.z_init+distance, save_image=save_image)

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

    def move_linear_trajectory(self, moving_step, num_step, corrector=0, save_image=False):
        pose = self.group.get_current_pose().pose
        for i in range(num_step):
            x = pose.position.x + (i+1)*moving_step
            y = pose.position.y + (i+1)*corrector
            self.move_linear(x, y, pose.position.z, save_image=save_image)

    def return_to_ready_pose(self):
        self.move_joint(0, -pi/4, 0, -3*pi/4, 0, pi/2, -pi/4)

    def show_position(self):
        print(self.group.get_current_pose().pose)

    # def update_pose(self):
        # self.pose = self.group.get_current_pose().pose

    def get_image(self, take_image):
        pose = self.group.get_current_pose().pose
        if take_image is True:
            filename = self.DataManager.generate_image_path(pose.position.x)
            self.Camera.save_image(filename)

    def show_image(self):
        self.Camera.show_image()

    def get_video_stream(self):
        self.Camera.get_video_stream()

    def close_camera(self):
        self.Camera.close()

    def calibration(self, moving_step, num_step):
        self.move_ready_pose()
        self.show_image()
        pose = self.group.get_current_pose().pose
        for i in range(num_step):
            x = pose.position.x + (i+1)*moving_step
            self.move_linear(x, pose.position.y, pose.position.z)
            self.show_image()
        self.return_to_ready_pose()

    def keyboard_control(self):
        pass

    def wait(self):
        rospy.sleep(self.sleep_time)
