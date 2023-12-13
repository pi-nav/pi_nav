#!/usr/bin/env python
import rospy
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float32

#Initial position also set in launch file.
coord = [[0,0], [3, -.2]]
global_coord = [[2.8, .13] , [1.6, .13], [.05, .13]]

#Intersection Center is offset to improve on late intersection exit detection. 
center = [1.47, -.35]
detect = .4

class RobotController:
    """
        Robot Controller for priority vehicle, subscribes to odometry and pose topics to get local and global positions.
        Publishes to robot#_vel for low level motor commands.
        
        For stop handling, publishes position status to emergency_stop topic. Other robots subscribe
        to this topic to give way to priority vehicle

        Attributes:
            sub_odom: Subscriber for robot odometry.
            sub_amcl: Subscriber for robot's global pose. Initial pose is set in launch file.
            sub_stop1: Subscriber for stop from other vehicle.
            sub_emergency_stop: Subscriber for stop signals.
            pub_vel: Publisher for robot's velocity commands.
            pub_stop2: Publisher for robot 
            pos: Local position of the robot.
            amcl_pos: Global position of the robot.
            speed_x: Linear speed of the robot.
            hz: Frequency of the control loop.
            stop: Flag to stop the robot.
            stop_time1: Time vehicle1 spends waiting.
            stop_time2: Time vehicle2 spends waiting.
            center: Coordinates of an intersection center.
            detect: Detection threshold for intersection.
            coord: Local coordinates for the robot.
            global_coord: Global coordinates for the robot.
    """
    def __init__(self, coord, global_coord):
        self.sub_odom = rospy.Subscriber("robot_3_odom", Odometry, self.odom_callback)
        self.sub_amcl = rospy.Subscriber("robot_3_amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.sub_stop1 = rospy.Subscriber('int_stop1', Float32, self.sub_stop_callback)
        self.sub_emergency_stop = rospy.Subscriber('emergency_stop1', Bool, self.emergency_stop_callback)

        self.pub = rospy.Publisher('robot_3_vel', Twist, queue_size=10)
        self.pub_stop2 = rospy.Publisher('int_stop2', Float32, queue_size=10)
        self.pub_e_stop = rospy.Publisher('emergency_stop2', Bool, queue_size=10)



        self.pos = None
        self.amcl_pos = None
        self.speed_x = 0.15
        self.hz = 10
        self.stop = None
        self.stop_time1 = None
        self.stop_time2 = 0


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

    def sub_stop_callback(self, msg):
        '''
            Callback for stop signal from other robot
        '''
        self.stop_time1 = msg.data

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
            print("Robot Continues")

    def get_pos(self):
        '''
            Retrieve robot local coordinates. Returns starting point if /robot_#_odom is not
            responding properly.          
        '''
        if self.pos is not None:
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

    def int_crossing(self):
        '''
            Sets bounding square with side length detect around intersection center. Publishes True to
            emergency_stop if priority vehicle is in the intersection.
        '''
        pos = self.get_pos()
        int_msg = Bool()
        if abs(self.center[0] - pos[0]) < self.detect and abs(self.center[1] - pos[1]) < self.detect:
            int_msg.data = True
        else:
            int_msg.data = False

        rospy.loginfo("Intersection Occupied: %s" % int_msg.data)
        self.pub_e_stop.publish(int_msg)
        
    def drive_adjust(self, pid_output):
        '''
            Publishes motor commands from PID controller to robot_#_vel topic. PID output will always be
            small angular velocities to correct robot path. 

            Parameters:
                pid_output: PID output, correcting angular velocities
        '''

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

        while not rospy.is_shutdown(): #and not self.stop:
            current_pos = self.get_pos()
            amcl_pos = self.get_amcl_pos()
           
            #Stops if /odom or /amcl_pose are not working
            if current_pos[0] is None or amcl_pos[0] is None:
                rate.sleep()
                continue

            #Robot has reached final goal
            if math.hypot(global_coord[2][0] - amcl_pos[0], global_coord[2][1] - amcl_pos[1]) < goal_threshold:
                print("Goal reached, stopping robot.")
                self.stop_robot()
                break 

            
            #Robot stops at mouth of intersection
            if math.hypot(global_coord[1][0] - amcl_pos[0], global_coord[1][1] - amcl_pos[1]) < goal_threshold:
                #Publish timer to /int_stop2 topic
                elapsed_time = self.time_since_stopped()
                int_msg = Float32()
                int_msg.data = float(elapsed_time)
                self.pub_stop2.publish(int_msg)
                #Wait 5 seconds at intersection if arrives first
                if elapsed_time == 0.0 or elapsed_time < 5:
                    self.stop_robot()
                    continue
                #If robot1 has waited longer than robot2, stop
                elif elapsed_time > 0.0 and self.stop_time1 > self.stop_time2:
                    self.stop_robot()
                #Checks if robot1 is crossing intersection
                elif self.stop_time1 > 5 and self.stop:
                    self.stop_robot()

            error = calc_cte(start_pos, end_pos, current_pos) 
            error_sum += error / self.hz
            error_diff = (error - error_prev) * self.hz
            error_prev = error

            output = Kp * error + Ki * error_sum + Kd * error_diff
            self.drive_adjust(output)

            rate.sleep()

    def straighten_robot(self):
        '''
            Straighten wheels of robot.
        '''
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        self.pub.publish(twist)
        print('Robot straightened')

    def stop_robot(self):
        '''
            Stop robot, start timer.
        '''
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        self.pub.publish(twist)

        if self.stop_time2 is 0:
            self.stop_time2 = time.time()
            print('Robot stopped. Timer Started')
        else:
            print('Robot stopped. Timer already running')

    def time_since_stopped(self):
        '''
            Calculate time spent stopped at intersection.
        '''
        if self.stop_time2 != 0:
            count = time.time() - self.stop_time2
            print(count)
            return count
        else:
            return 0.0



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
    rospy.init_node('robot_controller_node3')
    controller = RobotController(coord, global_coord)
    rospy.sleep(1)
    controller.straighten_robot()
    rospy.sleep(1)


    controller.pid_test(coord[0], coord[1])

