#!/usr/bin/env python

import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

x = 0
y = 0
yaw = 0


def print_pose(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = msg.twist.twist.angular.z


def move(speed, distance, is_forward):
    velocity_message = Twist()

    if (speed > 0.4):
        print('speed must be lower than 0.4')
        return

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)  # we publish the velocity at 10 Hz (10 times a second)
    cmd_vel_topic = 'cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = time.time()

    i = 0
    while True:
        i = i + 1
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()
        t1 = time.time()
        if i % 1000 is 0:
            print(t1)

        distance_moved = (t1 - t0) * speed
        print(distance_moved)
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    velocity_message = Twist()
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    loop_rate = rospy.Rate(10)  # we publish the velocity at 10 Hz (10 times a second)
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * angular_speed_degree
        loop_rate.sleep()

        print('current_angle_degree: ', current_angle_degree)

        if (current_angle_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def move_to_goal(speed, relative_angle_degree, distance):
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
        velocity_message.angular.z = abs(angular_speed)
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * 5
        loop_rate.sleep()

        if (current_angle_degree > relative_angle_degree):
            velocity_message.angular.z = 0
            break

    t0 = time.time()

    while True:
        rospy.loginfo("Turtlebot move forward")
        velocity_message.linear.x = speed
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()
        t1 = time.time()

        distance_moved = (t1 - t0) * speed

        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    velocity_message.angular.z = 0
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def move_to_goal_together(speed, angle, distance):
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    angular_speed = math.radians(angle)

    loop_rate = rospy.Rate(10)  # we publish the velocity at 10 Hz (10 times a second)
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    time_needed = distance / speed
    angle_per_time = angular_speed/time_needed

    while True:
        velocity_message.linear.x = speed
        velocity_message.angular.z = angle_per_time
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        loop_rate.sleep()

        if (t1 - t0 > time_needed):
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            break

    velocity_message.angular.z = 0
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_move_to_goal', anonymous=True)
        listener = rospy.Subscriber('/odom', Odometry, print_pose)
        time.sleep(1.0)
        # rotate(10, 90, True)
        # move(0.1, 3, True)
        move_to_goal_together(0.1, 90, 3)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
