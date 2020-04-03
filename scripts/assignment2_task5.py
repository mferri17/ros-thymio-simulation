#!/usr/bin/env python

import rospy
import math
import numpy as np
from utils import ThymioController



class ThymioController_Task5(ThymioController):

    def run(self):
        """Controls the Thymio."""

        ### TASK 4 and 5 are handled the same

        while not rospy.is_shutdown():
            rospy.loginfo('%s Moving forward...' % self.name)

            while not rospy.is_shutdown() and not self.collision():
                self.move(0.15, np.random.uniform(-2,+2))
                self.rate.sleep()
            
            self.stop()
            rospy.loginfo('%s Collision: trying to find a free path...' % self.name)

            while not rospy.is_shutdown() and self.collision():
                if self.is_exactly_facing_an_obstacle(tollerance = 0.05): # obstacle in front of the robot
                    self.rotate(180)
                elif self.proximity_right.range <= self.proximity_left.range: # obstacle at right
                    self.move(0, 1) # turning left
                else: # obstacle at left
                    self.move(0, -1) # turning right

            self.stop()




if __name__ == '__main__':

    controller = ThymioController_Task5('assignment2_task5')

    try:

        controller.run()


    except rospy.ROSInterruptException as e:
        pass
