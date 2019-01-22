#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      linear  = 0
      angular = 0
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance

      # Number of laser mesurments per cycle            
      scanLen = len(scan)

      # Angles of the laser mesurments in rads (0 rads is where the robot facing)
      angles = [math.radians(-135 + x*270/(scanLen - 1)) for x in range(scanLen)]

      # Calculate the speeds
      linear = -sum([math.cos(angles[x]) / scan[x]**2 for x in range(scanLen)])
      angular = -sum([math.sin(angles[x]) / scan[x]**2 for x in range(scanLen)])

      # print linear, angular
      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.
        
        # Combine the tho sets of speeds
        self.linear_velocity  = l_goal + l_laser**3 * 3e-11
        self.angular_velocity = a_goal + a_laser**3 * 3e-11
        
        # Make sure speeds are on the range [-3,3]
        if abs(self.linear_velocity) > 0.3:
          self.linear_velocity = 0.3 * cmp(self.linear_velocity, 0)

        if abs(self.angular_velocity) > 0.3:
          self.angular_velocity = 0.3 * cmp(self.angular_velocity, 0)
        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        
        # Make sure robot goes forward when it can
        l_laser += 300

        # Smoother steering
        a_laser /= 500

        # Make sure speeds are on the range [-3,3]
        if abs(l_laser) > 0.3:
          self.linear_velocity = 0.3 * cmp(l_laser, 0)
        else:
          self.linear_velocity = l_laser

        if abs(a_laser) > 0.3:
          self.angular_velocity = 0.3 * cmp(a_laser, 0)
        else:
          self.angular_velocity = a_laser
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
