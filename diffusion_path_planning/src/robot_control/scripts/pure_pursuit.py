#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import numpy as np
import threading
import math
from math import cos, sin, atan2, pi

from gz.transport13 import Node as gz_Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose

observation = np.array([0.0, 0.0, 0.0], float) #x, y, yaw angle

path = []

# Parameters
k = 0.1   # look forward gain
Lfc = 0.2 # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick

class State:

    def __init__(self):
        global observation
        self.yaw = observation[2]
        self.v = 0.0
        self.rear_x = observation[0]
        self.rear_y = observation[1]

    def update(self, a):
        global observation
        self.yaw = observation[2]
        self.v += a * commander.time_interval
        self.rear_x = observation[0]
        self.rear_y = observation[1]

        return self.v

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            if(ind > len(self.cx) - 1):
                ind = len(self.cx) - 1
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                print(f"ind;{ind}")
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break

                if (ind < len(self.cx) - 2):
                    ind = ind + 1 
                else:
                    ind

                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

class Commander(Node):

    def __init__(self):
        super().__init__('commander')

        self.wheel_vel= np.array([0,0,0,0], float) #rotation velocity of each wheel
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        self.time_interval = 0.02
        self.L = 0.125 # wheel base
        self.Rw = 0.03 # Radius of the front and rear wheel
        self.t = 0

        self.target_creation = False

        self.timer = self.create_timer(self.time_interval, self.timer_callback)

        self.target_speed = 0.5 # (m/s)

        # initial state
        self.state = State()

        self.first_step = True

        self.gz_node = gz_Node()
        self.timeout = 500

        self.set_pose_service_name = "/world/empty/set_pose"
        self.Pose_request =  Pose()
        self.Pose_request.name = "omni_robot"
        self.Pose_request.position.x = 0.0
        self.Pose_request.position.y = 0.0
        self.Pose_request.position.z = 0.1
        self.Pose_request.orientation.x = 0.0
        self.Pose_request.orientation.y = 0.0
        self.Pose_request.orientation.z = 0.0
        self.Pose_request.orientation.w = 1.0

    def pure_pursuit_steer_control(self, state, trajectory, pind):
        global observation, path
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1
            
        delta_x = state.rear_x - tx
        delta_y = state.rear_y - ty

        target_angle = -math.atan2(delta_y, delta_x)
        alpha = target_angle  - state.yaw
            
        return alpha, ind, Lf

    def proportional_control(self, target, current):
        a = Kp * (target - current)
        return a

    def timer_callback(self):
        global observation, path

        if(len(path) > 0):

            if(self.first_step):
                self.first_step = False

                angle = atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
                self.Pose_request.position.x = path[0][0]
                self.Pose_request.position.y = path[0][1]
                self.Pose_request.orientation.z = sin(angle/2)
                self.Pose_request.orientation.w = cos(angle/2)
                self.gz_node.request(self.set_pose_service_name, self.Pose_request, Pose, Boolean, self.timeout)
                
            if(self.target_creation == False):
                cx_list = []
                cy_list = []
                for i in range(len(path)):
                    cx_list.append(path[i][0])
                cx = np.array(cx_list)
                for i in range(len(path)):
                    cy_list.append(path[i][1])
                cy = np.array(cy_list)
                self.target_course = TargetCourse(cx, cy)
                self.target_ind, _ = self.target_course.search_target_index(self.state)
                self.target_creation = True

            ai = self.proportional_control(self.target_speed, self.state.v)
            alpha, self.target_ind, Lf = self.pure_pursuit_steer_control(self.state, self.target_course, self.target_ind)
            vel_y = -1*self.state.update(ai)
            vel_x = 0 

            if(self.target_ind < len(path) - 1):
                omega =  alpha / Lf
                self.wheel_vel[0] = (vel_x*math.sin(pi/4       ) + vel_y*math.cos(pi/4       ) + self.L*omega)/self.Rw
                self.wheel_vel[1] = (vel_x*math.sin(pi/4 + pi/2) + vel_y*math.cos(pi/4 + pi/2) + self.L*omega)/self.Rw
                self.wheel_vel[2] = (vel_x*math.sin(pi/4 - pi)   + vel_y*math.cos(pi/4 - pi)   + self.L*omega)/self.Rw
                self.wheel_vel[3] = (vel_x*math.sin(pi/4 - pi/2) + vel_y*math.cos(pi/4 - pi/2) + self.L*omega)/self.Rw
            else: # goal
                self.wheel_vel[0] = 0
                self.wheel_vel[1] = 0         
                self.wheel_vel[2] = 0
                self.wheel_vel[3] = 0          

            wheel_vel_array = Float64MultiArray(data=self.wheel_vel)    
            self.publisher_vel.publish(wheel_vel_array)  

class Get_modelstate(Node):

    def __init__(self):
        super().__init__('get_modelstate')
        self.subscription = self.create_subscription(
            TFMessage,
            '/model/omni_robot/pose',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global observation

        for tfsf in data.transforms:
            if(tfsf.child_frame_id == 'omni_robot'):
                pose = tfsf.transform.translation
                orientation = tfsf.transform.rotation

                observation[0] = pose.x
                observation[1] = pose.y
                q0 = orientation.x
                q1 = orientation.y
                q2 = orientation.z
                q3 = orientation.w

                numerator = q0*q1 + q2*q3
                denominator = q0**2 - q1**2 - q2**2 + q3**2

                if(numerator > 0):
                    observation[2] = pi - math.atan2(2*numerator, denominator)
                elif(numerator <= 0):
                    observation[2] = -pi - math.atan2(2*numerator, denominator)

class Get_path(Node):

    def __init__(self):
        super().__init__('get_path')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/path',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global path

        path_num = np.array(msg.data)
        path = path_num.reshape([int(path_num.shape[0]/2), 2])
        print(path)

if __name__ == '__main__':
    rclpy.init(args=None)

    commander = Commander()
    get_modelstate = Get_modelstate()
    get_path = Get_path()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(get_modelstate)
    executor.add_node(get_path)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

