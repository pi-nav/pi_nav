#!/usr/bin/env python
import rospy
from std_msgs.msg import String  # Import the String message type for emergency_stop

def publish_stop_command():
    # Initialize the node
    rospy.init_node('emergency_stop_publisher', anonymous=True)

    # Publisher for the emergency_stop topic
    emergency_stop_pub = rospy.Publisher('emergency_stop', String, queue_size=10)

    # Wait for 3 seconds
    rospy.sleep(3)

    # Publish "STOP" message to the emergency_stop topic
    stop_command = String()
    stop_command.data = "STOP"
    emergency_stop_pub.publish(stop_command)
    rospy.loginfo("Published STOP command to emergency_stop")

if __name__ == '__main__':
    try:
        publish_stop_command()
    except rospy.ROSInterruptException:
        pass
