#!/usr/bin/env python

import rospy
import math
import numpy as np
from utils import ThymioController



class ThymioController_Task1(ThymioController):

    def run(self, desired_radius = 0.25):
        """Controls the Thymio."""

        rospy.loginfo('%s Following an 8 trajectory...' % self.name)

        linear_speed = 0.14
        angular_speed = linear_speed / desired_radius
        circumference = 2 * np.pi * desired_radius

        time_needed = (circumference / linear_speed)

        rospy.loginfo('%s Time needed for completing a circle: %f' % (self.name, time_needed))
        time_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            self.move(linear_speed, angular_speed)

            if (rospy.Time.now().to_sec() - time_start) > time_needed:
                time_start = rospy.Time.now().to_sec()
                angular_speed = -angular_speed




if __name__ == '__main__':

    controller = ThymioController_Task1('assignment2_task1')

    try:

        controller.run()


    except rospy.ROSInterruptException as e:
        pass
