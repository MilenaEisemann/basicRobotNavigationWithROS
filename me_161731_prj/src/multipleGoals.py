#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates#
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import atan2, pow, sqrt
from goal_publisher.msg import PointArray

from std_srvs.srv import Empty, EmptyResponse

class robot(object):

    def __init__(self):
        rospy.init_node ('subscriber')
        print("initiating")

        #robots position & orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 99

        #rate for sleep
        self.r = rospy.Rate(10)

        self.goals = []             #array of all goals
        self.noGoals = 0            #number of all goals to reach
        self.currentGoal = 0        #index for next goal

        #position of current goal
        self.goal = Point()
        self.goal.x = 99            #initiate with 99 so robot doesn't immediately see it as goal 0,0 reached
        self.goal.y = 99

        #regions of LaserScan
        self.regions = {
            'frontLeft': 99,       #99 instead of inf
            'frontRight':99
        }

        self.state = 0

        self.subPos = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStates)
        self.subLaser = rospy.Subscriber('scan', LaserScan, self.callbackLaser)

        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    #set position of robot (x and y) and current angle theta
    def modelStates (self, msg):
        self.x = msg.pose[1].position.x
        self.y = msg.pose[1].position.y
        rot_q = msg.pose[1].orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # get minimal distance to obstacle in two regions in front of robot
    # frontLeft from laser 0-14
    # frontRight from laser 344-359
    def callbackLaser (self, msg):
        self.regions = {
            'frontLeft': min(min(msg.ranges[0:14]), 99),            # 99 if out of range
            'frontRight': min(min(msg.ranges[344:359]), 99)
        }

    # robot hat reached current goal
    # check if it already reached all goals or if there is another one
    def goal_reached(self):
        if self.currentGoal + 1 < self.noGoals:
            print "I reached goal nr " + str(self.currentGoal + 1)
            self.set_goal()
        else:
            print "hoooray...I reached all goal points"

    # change currentGoal to next one in array of all goals
    def set_goal(self):
        self.currentGoal = self.currentGoal + 1
        self.goal.x = self.goals[self.currentGoal].x
        self.goal.y = self.goals[self.currentGoal].y
        print "Next goal is goal nr " + str(self.currentGoal + 1) + " with the coordinates: " + str(self.goal)


    # main
    def run(self):
        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y

        speed = Twist()

        #state 0 --> go to point
        if self.state == 0:

            angle_to_goal = atan2 (inc_y, inc_x)
            if sqrt(pow(inc_x, 2) + pow(inc_y,2)) < 0.5:  #diagonal distance to goalpoint --> pythagoras
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.goal_reached()
            elif abs(angle_to_goal - self.theta) > 0.15:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            else:
                speed.linear.x = 0.3
                speed.angular.z = 0.0

            #if obstacle:
            if self.regions['frontLeft'] < 0.7 or self.regions['frontRight'] < 0.7:
                print "there seems to be an obstacle"
                self.state = 1      # --> switch to new state
                #else continue driving straight towards goal

        #state 1 --> instead of linear motion new state
        if self.state == 1:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            print "i should switch my mode"

        self.pubVel.publish(speed)



if __name__ == '__main__':
    #instantiate the class and set some parameters
    app = robot()

    msg = rospy.wait_for_message('goals', PointArray)   #wait until first message of 'goals'
    app.goals = msg.goals                               #fill array with all goals from message
    app.noGoals = len(app.goals)                        #get number of goals to reach
    app.goal.x = app.goals[app.currentGoal].x           #start with first goal at index 0 of goals array
    app.goal.y = app.goals[app.currentGoal].y

    print("I am starting to navigate to my goal points")

    #run the script in an infite loop to continously read and process laserdata
    while(True):
        app.run()
        app.r.sleep()
