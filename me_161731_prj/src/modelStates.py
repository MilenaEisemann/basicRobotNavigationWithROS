#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import atan2, sin, cos, pow, sqrt

#start is x:0, y:0
x = 0.0
y = 0.0
theta = 0.0     #current angle of robot

move_forward = False

def callback(msg):
    global x
    global y
    global theta

    x = msg.pose[1].position.x
    y = msg.pose[1].position.y
    rot_q = msg.pose[1].orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node ('subscriber')
sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 2
goal.y = 2

while not rospy.is_shutdown():
    inc_x = goal.x - x                      #distance robot to goal in x
    inc_y = goal.y - y                      #distance robot to goal in y
    angle_to_goal = atan2 (inc_y, inc_x)    #calculate angle through distance from robot to goal in x and y
    dist = sqrt(pow(inc_x, 2) + pow(inc_y, 2))      #calculate distance

    # find out which turndirection is better
    # the bigger the angle, the bigger turn, - when clockwise
    turn = atan2(sin(angle_to_goal-theta), cos(angle_to_goal-theta))

    if abs(angle_to_goal - theta) < 0.1:    #0.1 because it too exact for a robot if both angles should be exactly 0
        move_forward = True

    speed.angular.z = 0.2 * turn

    if move_forward == True:
        #keep speed between 0.3 and 0.7
        if 0.1 * dist > 0.3 and 0.1 * dist < 0.7:
            speed.linear.x = 0.05 * dist
        elif 0.1 * dist > 0.7:
            speed.linear.x = 0.7
        else:
            speed.linear.x = 0.3


    pub.publish(speed)
    r.sleep()
