#!/usr/bin/env python3
#Importiere Bibliotheken

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('IRP_Python', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# Hauptprogramm
if __name__ == '__main__':
    try:
        #Erstellung der Trajektorie
        path = [[0.2, -0.2, 0.5], [0.5, -0.2, 0.5], [0.5, 0.2, 0.5], [0.2, 0.2, 0.5], [0.2, -0.2, 0.5]]

        for i in range(len(path)):
            #Definiere die Zielposition
            home_pose=group.get_current_pose().pose
            pose=copy.deepcopy(home_pose)
            pose.position.x=path[i][0]
            pose.position.y=path[i][1]
            pose.position.z=path[i][2]

            #Plane die Wegpunkte
            waypoints = []
            waypoints.append(copy.deepcopy(pose))
            (plan, fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)         
            
            #Fuehre die Bewegung aus
            group.execute(plan, wait=True)

    except rospy.ROSInterruptException:
        pass