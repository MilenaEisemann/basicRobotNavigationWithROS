#! /usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan

from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import atan2

from goal_publisher.msg import PointArray

x = 0.0
y = 0.0
theta = 0.0

goalNr = 0

goal = Point()
stop = 0

#set position of robot (x and y) and current angle theta
def modelStates (msg):
    global x
    global y
    global theta
    global stop

    x = msg.pose[1].position.x
    y = msg.pose[1].position.y
    rot_q = msg.pose[1].orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #print "theta"
    #print theta

# get next point of goal_publisher
def callback(msg):  #korrekte Werte
    goal.x = msg.goals[0].x
    goal.y = msg.goals[0].y
    #print goal.x
    #print goal.y

def callbackLaser(msg):
    scan_data = msg.ranges
    for i in range(5):
        if scan_data[i] >= 0.2:
            print scan_data[i]
        else:
            print "there seems to be an obstacle"
            stop = stop+1
        # if scan_data[i] < 0.5:
        #     print "there seems to be an obstacle"
        #     stop = stop+1
    # for i in range(5):
    #     if scan_data[359-i] < 0.2:
    #         print "there seems to be an obstacle"
    #         stop = stop+1
    for i in range(5):
        if scan_data[359-i] >= 0.2:
            print scan_data[359-i]
        elif scan_data != float ('inf'):
            print "there seems to be an obstacle"
            stop = stop+1

rospy.init_node ('subscriber')

rospy.Subscriber('goals', PointArray, callback)
sub = rospy.Subscriber('/gazebo/model_states', ModelStates, modelStates)
subLaser = rospy.Subscriber('scan', LaserScan, callbackLaser)
pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


speed = Twist()

r = rospy.Rate(4)
inc_x = goal.x - x
inc_y = goal.y - y
print inc_y

while not rospy.is_shutdown():
    #import ipdb; ipdb.set_trace()
    inc_x = goal.x - x
    inc_y = goal.y - y

    print "goal is"
    print goal
    print "false"
    print stop

    angle_to_goal = atan2 (inc_y, inc_x)
    if stop == 0:
        if inc_x < 0.5 and inc_y <0.5:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            #print "I reached goal nr " + str(goalNr + 1)
            #goalNr=goalNr + 1
            print "I reached goal nr 1"
        elif abs(angle_to_goal - theta) > 0.2:
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        print "I stopped"

    pubVel.publish(speed)
    r.sleep()
