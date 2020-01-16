#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import atan2, sin, cos, pow, sqrt
from goal_publisher.msg import PointArray

from std_srvs.srv import Empty, EmptyResponse

class Robot(object):

    def __init__(self):
        rospy.init_node ('subscriber')
        print("initiating...")

        #starting point
        self.currentStart = Point()
        self.currentStart.x = 0.0
        self.currentStart.y= 0.0

        #robot's current position & orientation
        self.pos = Point()
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.theta = 99

        #rate for sleep
        self.r = rospy.Rate(10)

        #goals
        self.goals = []             #array of all goals
        self.noGoals = 0            #number of all goals to reach
        self.currentGoal = 0        #index for next goal

        #position of current goal
        self.goal = Point()
        self.goal.x = 99            #initiate with 99 so robot doesn't immediately see it as goal 0,0 reached
        self.goal.y = 99

        #variables for turning
        self.angle_to_goal = 0.0
        self.turn = 0.0
        self.direction = 1.0        #turndirection counterclockwise or clockwise

        #regions of LaserScan
        self.regions = {
            'frontLeft': 99,       #99 instead of inf
            'frontRight':99,
            'farRight': 99,
            'farLeft': 99
        }

        self.nearest = 99

        self.move_forward = False
        self.following_wall = False  #True if driving around obstacle

        self.back_on_track = False   #True at intersection of wall_follow-path and line from start to goal

        # position where last obstacle was detected
        self.obstacleEncountered = Point()
        self.obstacleEncountered.x = 0.0
        self.obstacleEncountered.x = 0.0

        # distance between self.obstacleEncountered and current location of robot
        self.dist = 0.0

        # 0 : no obstacle --> drive straight towards goal coordinates
        # 1 : obstacle encountered --> turn and drive around it
        self.state = 0

        self.subPos = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStates)
        self.subLaser = rospy.Subscriber('scan', LaserScan, self.callbackLaser)

        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    #set position of robot (x and y) and current angle theta
    def modelStates (self, msg):
        self.pos.x = msg.pose[1].position.x
        self.pos.y = msg.pose[1].position.y
        rot_q = msg.pose[1].orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # get minimal distance to obstacle in regions in front and sides of robot
    # frontLeft from laser 0-14
    # frontRight from laser 344-359
    # etc
    def callbackLaser (self, msg):
        self.regions = {
            'frontLeft': min(min(msg.ranges[0:14]), 99),            # 99 if out of range
            'frontRight': min(min(msg.ranges[344:359]), 99),
            'farRight': min(min(msg.ranges[271:276]), 99),
            'farLeft': min(min(msg.ranges[84:89]),99)
        }

    # robot hat reached current goal
    # check if it already reached all goals or if there is another one
    def goal_reached(self):
        self.currentStart.x = self.pos.x
        self.currentStart.y = self.pos.y
        self.move_forward = False

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
        print "Next goal is goal nr " + str(self.currentGoal + 1) + " with the coordinates: \n" + str(self.goal)

    #checks if current position of robot is on line between its starting_point and current goal
    def isBetweenStartAndGoal(self):
        # vector from current position to start
        dx1 = self.pos.x - self.currentStart.x
        dy1 = self.pos.y - self.currentStart.y

        # vector from goal to start
        dx2 = self.goal.x - self.currentStart.x
        dy2 = self.goal.y - self.currentStart.y

        cross = dx1 * dy2 - dy1 * dx2

        if cross < 0.0 + 0.3 and cross > 0.0 - 0.3:         #if crossproduct is 0 (+/- 0.1), the two vectors are collinear/in one line
            self.back_on_track = True
        else:
            self.back_on_track = False

    # calculate distance between self.obstacleEncountered and current position
    # reason: should not detect self.obstacleEncountered as a point between start and goal and therefore switch mode
    def drove_some_distance(self):
        self.dist = sqrt(pow(self.pos.x - self.obstacleEncountered.x, 2) + pow(self.pos.y - self.obstacleEncountered.y,2))

    # main
    def run(self):
        delta_x = self.goal.x - self.pos.x     #distance robot to goal in x
        delta_y = self.goal.y - self.pos.y    #distance robot to goal in y

        speed = Twist()

        #state 0 --> go to point
        if self.state == 0:
            self.back_on_track = False      #reset to default

            self.angle_to_goal = atan2 (delta_y, delta_x)       #calculate angle through distance from robot to goal in x and y
            dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2))      #calculate distance

            # find out which turn direction is better
            # the bigger the angle, the bigger turn, - when clockwise
            self.turn = atan2(sin(self.angle_to_goal-self.theta), cos(self.angle_to_goal-self.theta))

            if sqrt(pow(delta_x, 2) + pow(delta_y,2)) < 0.5 and self.regions['frontLeft'] > 0.7 and self.regions['frontRight'] > 0.7:  #diagonal distance to goalpoint --> pythagoras & check if front is clear
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.goal_reached()
            else:
                if abs(self.angle_to_goal - self.theta) < 0.2:    #0.2 because it too exact for a robot if both angles should be exactly 0
                    self.move_forward = True

                speed.angular.z = 0.2 * self.turn

                if self.move_forward == True:
                    #keep speed between 0.4 and 0.5
                    if 0.1 * dist > 0.4 and 0.1 * dist < 0.5:
                        speed.linear.x = 0.1 * dist
                    elif 0.1 * dist > 05:
                        speed.linear.x = 0.5
                    else:
                        speed.linear.x = 0.4

            #if obstacle detected:
            if (self.regions['frontLeft'] < 0.7 or self.regions['frontRight'] < 0.7) and self.move_forward == True:
                print "there seems to be an obstacle"
                self.nearest = min(self.regions['frontLeft'], self.regions['frontRight'])
                self.move_forward = False
                self.state = 1      # --> switch to new state

        #state 1 --> drive around walls of obstacle
        if self.state == 1:
            #turn 90 degrees to obstacle
            if self.following_wall == False:
                if abs(self.regions['farRight'] - self.nearest <= 0.03):
                    self.obstacleEncountered.x = self.pos.x
                    self.obstacleEncountered.y = self.pos.y

                    speed.linear.x = 0.3
                    speed.angular.z = 0.0
                    self.following_wall = True
                else:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.7

            #drive around obstacle
            #correct angular.z if too far or too close
            elif self.following_wall == True:
                self.isBetweenStartAndGoal()
                self.drove_some_distance()

                if self.regions['frontLeft'] > 0.7 and self.regions['frontRight'] > 0.7:
                    if self.back_on_track == True and self.dist > 1:
                        print "I'm back on track"
                        self.state = 0  # --> switch to new state
                        self.following_wall = False
                    else:
                        speed.angular.z = 0.9 * (0.2-self.regions['farRight'])
                        speed.linear.x = 0.3
                else:
                    speed.angular.z = 0.3
                    speed.linear.x = 0.3 * min(self.regions['frontLeft'], self.regions['frontRight'])
                if speed.angular.z > 1:
                    speed.angular.z = 0.9
                elif speed.angular.z < -1:
                    speed.angular.z = -0.9

        self.pubVel.publish(speed)



if __name__ == '__main__':
    #instantiate the class and set some parameters
    app = Robot()

    print "receiving goal points..."
    msg = rospy.wait_for_message('goals', PointArray)   #wait until first message of 'goals'
    app.goals = msg.goals                               #fill array with all goals from message
    app.noGoals = len(app.goals)                        #get number of goals to reach
    app.goal.x = app.goals[app.currentGoal].x           #start with first goal at index 0 of goals array
    app.goal.y = app.goals[app.currentGoal].y
    print "First goal is goal nr 1 with the coordinates: \n" + str(app.goal)

    print "I am starting to navigate to my goal points"

    #run the script in an infite loop to continously read and process laserdata
    while(True):
        app.run()
        app.r.sleep()
