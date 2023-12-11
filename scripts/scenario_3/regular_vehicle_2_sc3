#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool

#Initial position also set in launch file.
coord = [[0,0], [3, -.2]]
global_coord = [[0.05, -0.13] , [2.8, -0.13]]

#Intersection Center is offset to improve on late intersection exit detection. 
center = [1.05, 0]
detect = .4

class RobotController:
    """
        Robot Controller for priority vehicle, subscribes to odometry and pose topics to get local and global positions.
        Publishes to robot#_vel for low level motor commands. Subscribes to emergency_stop and regular_stop for stop commands.

        Attributes:
            sub_odom: Subscriber for robot odometry.
            sub_amcl: Subscriber for robot's global pose. Initial pose is set in launch file.
            pub_vel: Publisher for robot's velocity commands.
            sub_emergency_stop: Subscriber for stop signals.
            pub_regular_stop: publisher for regular vehicle stop signals
            sub_regular_stop: subscriber for regular vehicle stop signals
            pos: Local position of the robot.
            amcl_pos: Global position of the robot.
            speed_x: Linear speed of the robot.
            hz: Frequency of the control loop.
            stop: Flag to stop the robot.
            center: Coordinates of an intersection center.
            detect: Detection threshold for intersection.
            coord: Local coordinates for the robot.
            global_coord: Global coordinates for the robot.
    """
    def __init__(self, coord, global_coord):
        self.sub_odom = rospy.Subscriber("robot_2_odom", Odometry, self.odom_callback)
        self.sub_amcl = rospy.Subscriber("robot_2_amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.sub_emergency_stop = rospy.Subscriber('emergency_stop', Bool, self.emergency_stop_callback)

        self.pub = rospy.Publisher('robot_2_vel', Twist, queue_size=10)

        self.pub_regular_stop = rospy.Publisher('regular_stop', Bool, queue_size=10)
        self.sub_regular_stop = rospy.Subscriber('regular_stop', Bool, self.regular_stop_callback)

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
        '''
            Callback function for robot odometry subscriber.
        '''
        self.pos = msg.pose.pose.position

    def amcl_callback(self, msg):
        '''
            Callback function for robot AMCL pose subscriber.
        '''

        self.amcl_pos = msg.pose.pose.position

    def emergency_stop_callback(self, msg):
        '''
            Callback function for emergency_stop subscriber. Waits for priority vehicle
            to output whether it is crossing the intersection.
        '''
        if msg.data == True:
            self.stop = True
            # self.reset_pid()
            self.stop_robot()
            print("Emergency Stop Activated")
        if msg.data == False:
            self.stop = False
            # print("Robot Continues")

    def regular_stop_callback(self, msg):
        '''
            Callback function for regular_stop subscriber. Robot stops only if it is not the one
            publishing to the topic and it is not between the intersection and the goal.
        '''
        if msg.data == True and not self.is_crossing_self() and not self.is_between_intersection_and_goal():
            self.stop = True
            self.stop_robot()
            print("Regular Stop Activated by Another Robot")
        elif msg.data == False:
            self.stop = False
            print("Regular Stop Deactivated")

    def get_pos(self):
        '''
            Retrieve robot local coordinates. Returns starting point if /robot_#_odom is not
            responding properly.
        '''
        if self.pos is not None:
            # print('Current Position: {}, {}'.format(self.pos.x, self.pos.y))
            return [self.pos.x, self.pos.y]
        else:
            print('Local Position Not Set')
            return self.coord[0]

    def get_amcl_pos(self):
        '''
            Retrieve robot global coordinates. Returns starting point if /robot_#_amcl_pose is not
            responding properly.
        '''
        if self.amcl_pos is not None:
            print('Global Position: {}, {}'.format(self.amcl_pos.x, self.amcl_pos.y))
            return [self.amcl_pos.x, self.amcl_pos.y]
        else:
            print('Global Position Not Set')
            return self.global_coord[0]

    def reset_pid(self):
        '''
            Resets PID error.
        '''
        self.error_sum = 0
        self.error_prev = 0

    def int_crossing(self):
        '''
            Sets bounding square with side length detect around intersection center. Publishes True to
            regular_stop if priority vehicle is in the intersection.
        '''
        pos = self.get_pos()
        int_msg = Bool()
        if abs(self.center[0] - pos[0]) < self.detect and abs(self.center[1] - pos[1]) < self.detect:
            # print('*******IN THE INTERSECTION*********')
            int_msg.data = True
        else:
            int_msg.data = False
        self.pub_regular_stop.publish(int_msg)

    def is_crossing_self(self):
        '''
            Check if the robot is currently publishing a crossing message.

            Returns:
                bool: True if the robot is currently crossing, False otherwise.
        '''
        pos = self.get_pos()
        return abs(self.center[0] - pos[0]) < self.detect and abs(self.center[1] - pos[1]) < self.detect

    def is_between_intersection_and_goal(self):
        """
        Check if the robot is between the center of the intersection and its end goal.

        Returns:
            bool: True if the robot is between the intersection center and end goal, False otherwise.
        """
        robot_pos = self.get_amcl_pos()
        return self.is_point_between(robot_pos, self.center, self.global_coord[1])

    @staticmethod
    def is_point_between(point, line_start, line_end):
        """
        Check if a point is between two other points on a line.

        Parameters:
            point (list): The point to check.
            line_start (list): The start point of the line.
            line_end (list): The end point of the line.

        Returns:
            bool: True if the point is between line_start and line_end, False otherwise.
        """
        def distance(p1, p2):
            return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

        total_dist = distance(line_start, line_end)
        dist_to_start = distance(point, line_start)
        dist_to_end = distance(point, line_end)

        return dist_to_start + dist_to_end <= total_dist

    def drive_adjust(self, pid_output):
        '''
            Publishes motor commands from PID controller to robot_#_vel topic. PID output will always be
            small angular velocities to correct robot path. 

            Parameters:
                pid_output: PID output, correcting angular velocities
        '''
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
        '''
            PID controller. Corrects error from path by outputting small angular velocities. 

            Parameters to tune that affect PID performance:
                Kp: Proportional gain
                Ki: Integral gain
                Kd: Derivative gain
                self.hz: Loop Frequency
                self.x_speed: Motor forward speed

            Inputs:
                start_pos: Starting point in global frame
                end_pos: End point in global frame
        '''
        Kp = .2  # Fixes current error
        Ki = .01  # Fixes steady state error
        Kd = 1  # Predicts and fixes future error

        error_sum = 0
        error_prev = 0
        goal_threshold = 0.5

        rate = rospy.Rate(self.hz)

        while not rospy.is_shutdown():
            if not self.stop:
                current_pos = self.get_pos()
                amcl_pos = self.get_amcl_pos()

                #Stops if /odom or /amcl_pose are not working
                if current_pos[0] is None or amcl_pos[0] is None:
                    rate.sleep()
                    continue

                #Robot has reached final goal
                if math.hypot(global_coord[1][0] - amcl_pos[0], global_coord[1][1] - amcl_pos[1]) < goal_threshold:
                    print("Goal reached, stopping robot.")
                    self.stop_robot()
                    break 

                #Actual PID math
                error = calc_cte(start_pos, end_pos, current_pos) 
                error_sum += error / self.hz
                error_diff = (error - error_prev) * self.hz
                error_prev = error

                output = Kp * error + Ki * error_sum + Kd * error_diff
                
                self.drive_adjust(output)
                self.int_crossing()
            else:
                self.stop_robot()

            rate.sleep()

    def stop_robot(self):
        '''
            Stops robot.
        '''
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        self.pub.publish(twist)
        print('Robot stopped.')

def calc_cte(start_point, end_point, robot_position):
    '''
        Calculate cross track error for PID Controller. Finds equation of line composed from start_point
        and end_point. Calculates signed perpendicular distance of robot_position, represented as a point, 
        to this path. Negative error is on "right" of path, positive error is on "left".

        Parameters:
            start_point: Global coordinates of robot start
            end_point: Global coordaintes of robot goal
            robot_position: Robot current position in global coordinates given by /amcl_pose

        Returns:
            signed_cte: Calculated Cross track error, signed according to path side.
    '''
    x1, y1 = start_point
    x2, y2 = end_point
    x, y = robot_position

    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2

    raw_cte = A * x + B * y + C

    cte_magnitude = abs(raw_cte) / math.sqrt(A**2 + B**2)

    # Determine direction of motor adjustment based on sign of perpendicular distance.
    det = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1)
    sign = -1 if det > 0 else 1 if det < 0 else 0
    signed_cte = sign * cte_magnitude

    print('Error from Path: {}'.format(signed_cte))
    return signed_cte

if __name__ == '__main__':
    rospy.init_node('robot_controller_node_2')
    controller = RobotController(coord, global_coord)
    rospy.sleep(1)
    controller.stop_robot()
    rospy.sleep(1)
    controller.pid_test(coord[0], coord[1])