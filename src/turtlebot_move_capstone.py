#!/usr/bin/env python

import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

x = 0
y = 0
size = 0

def print_deepsort(msg):
    global x, y, size
    x, y, size = msg.data.split(',')
    x = float(x)
    y = float(y)
    size = float(size)


def move_to_goal_together(speed, relative_angle_degree):
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    angular_speed = math.radians(5)

    loop_rate = rospy.Rate(10)  # we publish the velocity at 10 Hz (10 times a second)
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        if relative_angle_degree>0:
            velocity_message.angular.z = angular_speed
        elif relative_angle_degree<0:
            velocity_message.angular.z = -angular_speed
        else:
            velocity_message.angular.z = 0
        velocity_message.linear.x = speed
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        loop_rate.sleep()

        if (t1-t0>0.5):
            break

    velocity_message.angular.z = 0
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_capstone', anonymous=True)
        deepsort_listener = rospy.Subscriber('/deepsort_poition_topic', String, print_deepsort)
        time.sleep(1.0)
        # move_to_goal_together(0.1, -90, 1) # turn left 90 degrees and go forward 2m 
        # for i in range(4):
        #     move_to_goal_together(1, 5, 0.1)
        while True:
            if size > 50:
                speed = -0.1
                print('backward')
            elif size < 30:
                speed = 0.2
                print('forward')
            else:
                print('normal')
                speed = 0.1
            if x>65:
                move_to_goal_together(speed, -3)
                print(x, 'right')
            elif x<35:
                move_to_goal_together(speed, 3)
                print(x, 'left')
            else:
                move_to_goal_together(speed, 0)
                print(x, 'straight')
            
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
