#!/usr/bin/env python

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion



class ThymioController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node('thymio_controller_task3')

        self.name = rospy.get_param('~robot_name')
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

        # create subscribers
        self.pose_subscriber = rospy.Subscriber(self.name + '/odom', Odometry, self.log_odometry)
        self.proximity_center_subscriber = rospy.Subscriber(self.name + '/proximity/center', Range, self.update_proximity_center)
        self.proximity_left_subscriber = rospy.Subscriber(self.name + '/proximity/left', Range, self.update_proximity_left)
        self.proximity_right_subscriber = rospy.Subscriber(self.name + '/proximity/right', Range, self.update_proximity_right)
        
        rospy.on_shutdown(self.stop) # tell ros to call stop when the program is terminated

        self.pose = Pose() # initialize pose to (X=0, Y=0, theta=0)
        self.velocity = Twist() # initialize linear and angular velocities to 0
        self.proximity_center = Range()
        self.proximity_left = Range()
        self.proximity_right = Range()
        self.proximity_center.range = 1
        self.proximity_left.range = 1
        self.proximity_right.range = 1
        self.rate = rospy.Rate(10) # set node update frequency in Hz


    def update_proximity_center(self, data):
        self.proximity_center = data

    def update_proximity_left(self, data):
        self.proximity_left = data

    def update_proximity_right(self, data):
        self.proximity_right = data


    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result


    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        # printable_pose = self.human_readable_pose2d(self.pose)

        # # log robot's pose
        # rospy.loginfo_throttle(
        #     period=5,  # log every 10 seconds
        #     msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        # )


    def euclidean_distance_2d(self, goal_x, goal_y):
        """Euclidean distance between current pose and the goal."""
        (current_x, current_y, _) = self.human_readable_pose2d(self.pose)
        return math.sqrt(math.pow((goal_x - current_x), 2) +
                         math.pow((goal_y - current_y), 2))


    def move(self, linear_vel=0.15, angular_vel=0):
        """Moves the robot accordingly to given velocities."""

        velocity = Twist(
            linear=Vector3(linear_vel, 0, 0),
            angular=Vector3(0, 0, angular_vel)
        )
        self.velocity_publisher.publish(velocity)
        self.rate.sleep()

    def stop(self):
        """Stops the robot."""

        self.velocity_publisher.publish(Twist())  # set velocities to 0
        self.rate.sleep()

    def rotate(self, angle, speed=100):

        vel_msg = Twist()

        # Converting from angles to radians
        relative_angle = np.deg2rad(angle)
        angular_speed = np.deg2rad(speed)

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while not rospy.is_shutdown() and current_angle < relative_angle:
            self.move(0, angular_speed)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)


    def is_exactly_facing_an_obstacle(self, tollerance=0.003):
        return  self.proximity_left.range < self.proximity_left.max_range and \
                self.proximity_right.range < self.proximity_right.max_range and \
                abs(self.proximity_left.range - self.proximity_right.range) < tollerance


    def run(self):
        """Controls the Thymio."""

        ### TASK 2

        wall_distance = 0.08 # meters

        rospy.loginfo('Moving forward...')

        while not rospy.is_shutdown() and self.proximity_center.range >= wall_distance:
            self.move()
            self.rate.sleep()

        self.stop()
        rospy.loginfo('CHECKPOINT Task 2: Collision (%.2f meters distant from the wall)' % wall_distance)
        rospy.loginfo('Trying to face the wall exactly...')

        while not rospy.is_shutdown() and not self.is_exactly_facing_an_obstacle():
            if self.proximity_left.range <= self.proximity_right.range:
                self.move(0, 0.2) # turning left
            else:
                self.move(0, -0.2) # turning right

        self.stop()
        rospy.loginfo('CHECKPOINT Task 2: Facing the wall exactly')
        rospy.loginfo('Waiting a few seconds...')
        rospy.sleep(2) # sleep for 2 seconds

        ### TASK 3

        rospy.loginfo('Turning by 180 degrees...')
        self.rotate(180)

        self.stop()
        rospy.loginfo('CHECKPOINT Task 3: The wall is exactly behind')

        from_wall = 2
        (init_x, init_y, init_theta) = self.human_readable_pose2d(self.pose)
        goal_x = from_wall * math.cos(init_theta) + init_x
        goal_y = from_wall * math.sin(init_theta) + init_y

        rospy.loginfo('Start walking for having the wall %.2f meters far behind...' % from_wall)
        rospy.loginfo('Starting pose is: \t init_x %f \t init_y %f' % (init_x, init_y))

        while not rospy.is_shutdown() and self.euclidean_distance_2d(goal_x, goal_y) >= wall_distance:
            self.move()

        self.stop()

        (final_x, final_y, final_theta) = self.human_readable_pose2d(self.pose)
        rospy.loginfo('Final pose is: \t\t final_x %f \t final_y %f' % (final_x, final_y))
        rospy.loginfo('%f meters distant from the starting pose (the one near the wall)' % self.euclidean_distance_2d(init_x, init_y))
        rospy.loginfo('CHECKPOINT Task 3: This means the robot is almost 2 meters distant from the wall')




if __name__ == '__main__':

    controller = ThymioController()

    try:

        controller.run()


    except rospy.ROSInterruptException as e:
        pass
