#!/usr/bin/env python

import roslib
roslib.load_manifest('simulation_control')
import rospy
import actionlib
import mavros_state
import time
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String

from state_machine import StateMachine

from simulation_control.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal

from std_msgs.msg import Float32

# mv_state = None
# goto_position_client = None
# goto_position_goal = None




class PIDPrint:
    def __init__(self):
        self.vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)
        print('Setting offboard')
        mv_state = mavros_state.mavros_state()
        mv_state.set_mode('OFFBOARD')
        print('Arming vehicle')
        mv_state.arm(True)
        rospy.loginfo("Taking off")
        self.goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
        # Will return True or False
        self.goto_position_client.wait_for_server()
        self.goto_position_goal = goto_positionGoal()
        self.goto_position_goal.destination.pose.position.z = 3
        self.goto_position_client.send_goal(self.goto_position_goal)
        self.goto_position_client.wait_for_result()

        self.des_pose = PoseStamped()

        self.mode_control.publish('velctr')
        rospy.sleep(0.1)

        # Fly back and forth
        print(str(PoseStamped))
        self.des_pose.pose.position.x = 0
        self.des_pose.pose.position.y = 1
        self.des_pose.pose.position.z = 3
        self.vel_control.publish(self.des_pose)
        # while not self.target_reached:
        rospy.sleep(20)

        while True:
            print(str(PoseStamped))
            self.des_pose.pose.position.x = 0
            self.des_pose.pose.position.y = -2
            self.des_pose.pose.position.z = 3
            self.vel_control.publish(self.des_pose)
            # while not self.target_reached:
            rospy.sleep(20)

            print('IN WHILE222')
            self.des_pose.pose.position.x = 0
            self.des_pose.pose.position.y = 2
            self.des_pose.pose.position.z = 3
            self.vel_control.publish(self.des_pose)
            # while not self.target_reached:
            rospy.sleep(20)
            print('IN WHILE333')

        # fly_to_pos(0, 0, 5)

        # Commit landing for Drone
        # mv_state.land(0.0)

    def init_takeoff(self):
        print("asd")

    def fly_to_pos(self, x, y, z):
        self.goto_position_goal.destination.pose.position.x = x
        self.goto_position_goal.destination.pose.position.y = y
        self.goto_position_goal.destination.pose.position.z = z
        self.goto_position_client.send_goal(self.goto_position_goal)
        self.goto_position_client.wait_for_result()


    def distance_reached_cb(self, data):
        self.target_reached = data.data



if __name__ == '__main__':
    # Set variables
    rospy.init_node('action_controller')
    PIDPrint()
