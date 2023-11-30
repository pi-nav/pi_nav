#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3

def publish_forward_command():
    # Initialize the ROS node
    rospy.init_node('move_robot_forward', anonymous=True)

    # Create a publisher object for the /robot1/cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    gc_pub = rospy.Publisher('gc_vel', Twist, queue_size=10)

    # Wait for the publisher to establish a connection to the topic
    rospy.sleep(1)

    # Create a Twist message for forward movement
    forward_command = Twist()
    forward_command.linear = Vector3(0.15, 0, 0)  # Set linear velocity (forward)
    forward_command.angular = Vector3(0, 0, 0)   # Set angular velocity to zero

    # Publish the command
    pub.publish(forward_command)
    gc_pub.publish(forward_command)
    rospy.loginfo("cmd_vel")
    rospy.loginfo("gc_vel")

    # Wait for a short time to ensure the message is sent
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_forward_command()
    except rospy.ROSInterruptException:
        pass
