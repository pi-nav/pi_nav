#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String  # Import the String message type

class RobotController:
    def __init__(self):
        # CHANGE THESE TOPIC NAMES
        self.sub_cmd_vel = rospy.Subscriber('robot_1_vel', Twist, self.cmd_vel_callback)
        self.sub_emergency_stop = rospy.Subscriber('emergency_stop', String, self.emergency_stop_callback)  # Subscriber for emergency stop
        self.pub = rospy.Publisher('robot_1_vel', Twist, queue_size=10)

        self.speed_x = 0.15
        self.stop = False  # Flag for emergency stop

    def cmd_vel_callback(self, msg):
        print('Received Motor Commands: {} Linear, {} Angular'.format(msg.linear.x, msg.angular.z))

    def emergency_stop_callback(self, msg):
        if msg.data == "STOP":
            self.stop = True
            self.stop_robot()
            print("Emergency Stop Activated")

    def stop_robot(self):
        # Function to stop the robot
        stop_twist = Twist()
        stop_twist.linear = Vector3(0, 0, 0)
        stop_twist.angular = Vector3(0, 0, 0)
        self.pub.publish(stop_twist)
        print('Published Motor Commands: 0 Linear, 0 Angular')

    def drive_forward(self):
        if not self.stop:  # Check if the stop flag is False
            # Create a Twist message for forward movement
            twist = Twist()
            twist.linear = Vector3(self.speed_x, 0, 0)  # Set linear velocity (forward)
            twist.angular = Vector3(0, 0, 0)           # Set angular velocity to zero

            # Publish the forward movement command
            self.pub.publish(twist)
            print('Published Motor Commands: {} Linear, {} Angular'.format(self.speed_x, 0))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.drive_forward()
            rate.sleep()

if __name__ == '__main__':
    # CHANGE THE NODE NAME
    rospy.init_node('robot_1_controller_node')

    controller = RobotController()
    controller.run()  # Start the continuous movement
