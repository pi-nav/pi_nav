#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3

def publish_forward_command():
    # Initialize the node
    rospy.init_node('move_robots_forward', anonymous=True)

    # ADD PUBLISHERS FOR ALL THE ROBOTS HERE
    robot_1_pub = rospy.Publisher('robot_1_vel', Twist, queue_size=10)
    robot_2_pub = rospy.Publisher('robot_2_vel', Twist, queue_size=10)

    # Wait for the publisher to establish a connection to the topic
    rospy.sleep(1)

    # Create a Twist message for forward movement
    forward_command = Twist()
    forward_command.linear = Vector3(0.15, 0, 0)  # Set linear velocity (forward)
    forward_command.angular = Vector3(0, 0, 0)   # Set angular velocity to zero

    # Publish the command
    robot_1_pub.publish(forward_command)
    robot_2_pub.publish(forward_command)
    rospy.loginfo("robot_1_vel")
    rospy.loginfo("robot_2_vel")

    # Wait for a short time to ensure the message is sent
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_forward_command()
    except rospy.ROSInterruptException:
        pass
