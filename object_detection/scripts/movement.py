#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import signal
import sys

# Define the velocity commands
linear_speed = 0.05  # meters per second
angular_speed = 0.35  # radians per second
correction_factor = 0.0  # Correction factor for angular velocity

# Flag to indicate if the robot should continue moving
is_moving = True

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    global is_moving
    is_moving = False
    move_forward(0.0)
    rotate(0.0, 'left')
    rospy.loginfo("Ctrl+C pressed. Stopping the robot.")
    sys.exit(0)

# Calculate the duration required to move a specific distance with given speed
def calculate_duration(distance, speed):
    return distance / speed

# Move the robot forward for the specified distance
def move_forward(distance):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = linear_speed

    # Calculate the duration required to move forward
    duration = calculate_duration(distance, linear_speed)

    # Set the current time
    current_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - current_time) < duration and is_moving:
        vel_msg.angular.z = -correction_factor * vel_msg.linear.x  # Apply angular correction
        velocity_publisher.publish(vel_msg)

    # Stop the robot after reaching the desired distance
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

# Rotate the robot by the specified angle
def rotate(angle, direction="left"):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    if direction == "left":
        vel_msg.angular.z = angular_speed
    elif direction == "right":
        vel_msg.angular.z = -angular_speed
    else:
        rospy.logerr("Invalid direction parameter. Valid values are 'left' or 'right'.")
        return

    # Calculate the duration required to rotate
    duration = calculate_duration(angle, angular_speed)

    # Set the current time
    current_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - current_time) < duration and is_moving:
        velocity_publisher.publish(vel_msg)

    # Stop the robot after reaching the desired angle
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

# Main function
if __name__ == '__main__':
    rospy.init_node('turtlebot3_rectangle_trajectory')
    move_forward(0.0)
    rotate(0, "left")
    rospy.sleep(1)

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Move forward
    move_forward(2.0)
    rospy.sleep(1)
    
    # Rotate by -45 degrees, exeminate the kitchen
    rotate(1.5708/2, "right")  # -45 degrees in radians
    rospy.sleep(3)
    
    # Rotate by 45 degrees
    rotate(1.5708/2, "left")  # 45 degrees in radians
    rospy.sleep(1)
    
    # Move forward
    move_forward(3.5)
    rospy.sleep(1)

    # Rotate by 90 degrees
    rotate(1.5708, "left")  # 90 degrees in radians
    rospy.sleep(1)

    # Move forward enter the room with the white dummy
    move_forward(6.0)
    rospy.sleep(3)

    # Rotate by 90 degrees
    rotate(1.5708, "left")  # 90 degrees in radians
    rospy.sleep(1)

    # Move forward
    move_forward(0.5)
    rospy.sleep(1)

    # Rotate by 90 degrees
    rotate(1.5708, "left")  # 90 degrees in radians
    rospy.sleep(1)

    # Move forward
    move_forward(2.0)
    rospy.sleep(1)

    # Rotate by -90 degrees
    rotate(1.5708, "right")  # -90 degrees in radians
    rospy.sleep(1)

    # Move forward
    move_forward(2.0)
    rospy.sleep(1)

    # Rotate by -90 degrees
    rotate(-1.5708, "right")  # -90 degrees in radians
    rospy.sleep(1)

    # Move forward, enter the wakening
    move_forward(2.0)
    rospy.sleep(3)

    # Rotate by 90 degrees
    rotate(1.5708, "left")  # 90 degrees in radians
    rospy.sleep(1)

    # Move forward
    move_forward(6.0)
    rospy.sleep(1)

    # Rotate by 45 degrees, enter the living room
    rotate(1.5708/2, "left")  # 45 degrees in radians
    rospy.sleep(3)

    # Rotate by 45 degrees
    rotate(1.5708/2, "left")  # 45 degrees in radians
    rospy.sleep(1)

    # Move forward
    move_forward(6.0)
    rospy.sleep(1)

    # Rotate by 90 degrees
    rotate(1.5708, "left")  # 90 degrees in radians
    rospy.sleep(1)
    
    # Move forward into the origin
    move_forward(3.0)
    rospy.sleep(1)

    # Stop the robot
    rospy.loginfo("Rectangle trajectory complete. Stopping the robot.")
    rospy.sleep(1)

rospy.spin()