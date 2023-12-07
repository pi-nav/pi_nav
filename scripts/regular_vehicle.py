#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool

coord = [[0,0], [3, 0]]
global_coord = [[0, 0] , [2.8, 0]]

center = [1.05, 0]
detect = .4

class RobotController:
    def __init__(self, coord, global_coord):
        # CHANGE THESE TOPIC NAMES
        # self.sub_cmd_vel = rospy.Subscriber('robot_1_vel', Twist, self.cmd_vel_callback)
        # self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # self.sub_amcl = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        # self.sub_emergency_stop = rospy.Subscriber('emergency_stop', String, self.emergency_stop_callback)

        # self.pub = rospy.Publisher('robot_1_vel', Twist, queue_size=10)

        self.sub_odom = rospy.Subscriber("robot_2_odom", Odometry, self.odom_callback)
        self.sub_amcl = rospy.Subscriber("robot_2_amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.sub_emergency_stop = rospy.Subscriber('emergency_stop', Bool, self.emergency_stop_callback)

        self.pub = rospy.Publisher('robot_2_vel', Twist, queue_size=10)
        # self.pub_stop = rospy.Publisher('emergency_stop', Bool, queue_size=10)

        self.pos = None
        self.amcl_pos = None
        self.speed_x = 0.15
        self.hz = 2
        self.stop = False


        self.center = center
        self.detect = detect
        self.coord = coord
        self.global_coord = global_coord

    def odom_callback(self, msg):
        self.pos = msg.pose.pose.position

    def amcl_callback(self, msg):
        self.amcl_pos = msg.pose.pose.position

    def emergency_stop_callback(self, msg):
        if msg.data == True:
            self.stop = True
            self.reset_pid()
            self.stop_robot()
            print("Emergency Stop Activated")
        if msg.data == False:
            self.stop = False
            print("Robot Continues")

    def get_pos(self):
        if self.pos is not None:
            # print('Current Position: {}, {}'.format(self.pos.x, self.pos.y))
            return [self.pos.x, self.pos.y]
        else:
            print('Local Position Not Set')
            return self.coord[0]

    def get_amcl_pos(self):
        if self.amcl_pos is not None:
            print('Global Position: {}, {}'.format(self.amcl_pos.x, self.amcl_pos.y))
            return [self.amcl_pos.x, self.amcl_pos.y]
        else:
            print('Global Position Not Set')
            return self.global_coord[0]

    def reset_pid(self):
        self.error_sum = 0
        self.error_prev = 0

    def int_crossing(self):
        pos = self.get_pos()
        int_msg = Bool()
        if abs(self.center[0] - pos[0]) < self.detect and abs(self.center[1] - pos[1]) < self.detect:
            # print('*******IN THE INTERSECTION*********')
            int_msg.data = True
        else:
            int_msg.data = False

        rospy.loginfo("Intersection Occupied: %s" % int_msg.data)

        # Publish the message
        # self.pub_stop.publish(int_msg)

    def drive_adjust(self, pid_output):
        if not self.stop:
            if pid_output > 3:
                input = 3
            elif pid_output < -3:
                input = -3
            else:
                input = pid_output

            twist = Twist()
            twist.linear = Vector3(self.speed_x, 0, 0)  # Set linear speed
            twist.angular = Vector3(0, 0, input)  # Set angular speed
            print('Published Motor Commands: {} Linear, {} Angular'.format(self.speed_x, input))
            self.pub.publish(twist)

    def pid_test(self, start_pos, end_pos):
        Kp = .2  # Proportional gain
        Ki = .01  # Integral gain
        Kd = 1  # Derivative gain

        error_sum = 0
        error_prev = 0
        goal_threshold = 0.5

        rate = rospy.Rate(self.hz)

        while not rospy.is_shutdown():
            if not self.stop:
                current_pos = self.get_pos()
                amcl_pos = self.get_amcl_pos()
                # print('Robot Position: {} '.format(current_pos))
                # print(self.stop)

                if current_pos[0] is None or amcl_pos[0] is None:
                    rate.sleep()
                    continue

                # if math.hypot(end_pos[0] - current_pos[0], end_pos[1] - current_pos[1]) < goal_threshold:
                if math.hypot(global_coord[1][0] - amcl_pos[0], global_coord[1][1] - amcl_pos[1]) < goal_threshold:
                    print("Goal reached, stopping robot.")
                    self.stop_robot()
                    break 

                error = calc_cte(start_pos, end_pos, current_pos) 
                error_sum += error / self.hz
                error_diff = (error - error_prev) * self.hz
                error_prev = error

                output = Kp * error + Ki * error_sum + Kd * error_diff
                
            # if not self.stop:
                self.drive_adjust(output)
                self.int_crossing()
            else:
                self.stop_robot()
                # continue

            rate.sleep()

    def stop_robot(self):
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        self.pub.publish(twist)
        print('Robot stopped.')



def calc_cte(start_point, end_point, robot_position):
    x1, y1 = start_point
    x2, y2 = end_point
    x, y = robot_position

    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2

    raw_cte = A * x + B * y + C

    cte_magnitude = abs(raw_cte) / math.sqrt(A**2 + B**2)

    # Determine the sign of the CTE
    # Compute the determinant of a matrix formed by the line and the robot position
    det = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1)

    # Assign the sign based on the determinant
    sign = -1 if det > 0 else 1 if det < 0 else 0

    # Apply the sign to the CTE, Negative error on the right, Postive error on the left
    signed_cte = sign * cte_magnitude

    print('Error from Path: {}'.format(signed_cte))
    return signed_cte


if __name__ == '__main__':
    # CHANGE THE NODE NAME
    rospy.init_node('robot_controller_node_2')
    controller = RobotController(coord, global_coord)
    rospy.sleep(1)
    controller.stop_robot()
    rospy.sleep(1)

    controller.pid_test(coord[0], coord[1])


