#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import numpy as np
import os.path as osp
import copy
from datetime import datetime
import torch
import cv2
from pathlib import Path
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = True
import diffuser.utils as utils
from diffuser.guides.plan_pb_diff_helper import DiffusionPlanner
from diffuser.guides.rm2d_plan_env import RandMaze2DEnvPlanner

from PyQt5.Qt import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from path_visualization_window import Ui_Form

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

scale = 0.01 #m/pixel
map_size = 500 #pixel

class Parser(utils.Parser):
    dataset: str = None
    config: str

class Map_GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.mouse_x = 0
        self.mouse_y = 0
        self.click_mouse = False

    def mouseMoveEvent(self, event):
        self.mouse_x = event.scenePos().x()
        self.mouse_y = event.scenePos().y()

        dist2start = math.sqrt((self.mouse_x - window.start_point[0]/scale)**2 + (self.mouse_y - (map_size - window.start_point[1]/scale))**2)
        dist2goal = math.sqrt((self.mouse_x - window.goal_point[0]/scale)**2 + (self.mouse_y - (map_size - window.goal_point[1]/scale))**2)        

        if(self.click_mouse):
            if(dist2start < 6):
                window.start_point[0] = self.mouse_x*scale
                window.start_point[1] = (map_size - self.mouse_y)*scale
                window.repaint()
                #print(f"start, stop:{window.start_point}")

            if(dist2goal < 6):
                window.goal_point[0] = self.mouse_x*scale
                window.goal_point[1] = (map_size - self.mouse_y)*scale    
                window.repaint()

    def mousePressEvent(self, event):
        self.click_mouse = True 

    def mouseReleaseEvent(self, event):
        self.click_mouse = False

class Path_Visualization_Simulator(QDialog):
    def __init__(self,parent=None):
        super(Path_Visualization_Simulator, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.map_scene = Map_GraphicsScene(self.ui.map_graphicsView)
        self.ui.map_graphicsView.setScene(self.map_scene)
        self.ui.map_graphicsView.setMouseTracking(True)

        rclpy.init(args=None)
        self.pub_node = Node('pub_path')
        self.pub = self.pub_node.create_publisher(Float64MultiArray, '/path', 10)
        self.sub_node = Node('sub_observation')
        self.sub = self.sub_node.create_subscription(TFMessage, '/model/omni_robot/pose', self.listener_callback, 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)
        
        self.pixmap = QPixmap(map_size, map_size)
        self.vehicle_pen = QPen(Qt.red)
        self.center_line=QPen(Qt.black)
        self.center_line.setStyle(Qt.DashLine)
        self.path_line = QPen(Qt.black)
        self.path_line.setWidth(3)
        self.diameter = 2 #pix
        self.C = 10 #size of the arrow
        self.first_time = True
        self.dot_size = 12
        self.middle_dot_size = 6
        self.test_env_number = 0
        self.start_point = [0.5, 4.5]
        self.goal_point = [4.5, 0.5]

        self.dplanner = DiffusionPlanner(args_train, args)
        self.rm2d_planner = RandMaze2DEnvPlanner()
        self.repaint()

        self.path = []

    def draw_stage(self):

        self.map_scene.clear()
        for i in range(4):
            L = int(map_size/5)
            self.map_scene.addLine(QLineF(L*(i+1), 0, L*(i+1), map_size), self.center_line) 
            self.map_scene.addLine(QLineF(0, L*(i+1), map_size, L*(i+1)), self.center_line)

        self.test_env_number = self.ui.Env_Number_spinBox.value()
        obstacle_coordinates = self.dplanner.problems_dict['infos/wall_locations'][self.test_env_number][0]
        oc_2dim = np.array(obstacle_coordinates).reshape([int(len(obstacle_coordinates)/2), 2])
        
        if len(oc_2dim) == 3:
            side_length = int(1.4/scale)
        else:
            side_length = int(1.0/scale)

        for i in range((len(oc_2dim))):
            x1 = int((oc_2dim[i][0] - 0.7)/scale)
            y1 = int(map_size - (oc_2dim[i][1] + 0.7)/scale)
            self.map_scene.addRect(x1, y1, side_length, side_length,
                                   QPen(QColor(137,189,222,160)), QBrush(QColor(137,189,222,160)))

    def listener_callback(self, data):
        for tfsf in data.transforms:
            if(tfsf.child_frame_id == 'omni_robot'):
                pose = tfsf.transform.translation
                orientation = tfsf.transform.rotation
                x = pose.x/scale 
                y = map_size - pose.y/scale
                q0 = orientation.x
                q1 = orientation.y
                q2 = orientation.z
                q3 = orientation.w
                numerator = q0*q1 + q2*q3
                denominator = q0**2 - q1**2 - q2**2 + q3**2
                angle = -math.pi/2 + math.atan2(2*numerator, denominator)

                if(self.first_time == False):
                    self.map_scene.clear()
                    self.map_scene.addPixmap(self.pixmap)
                    self.ui.map_graphicsView.setScene(self.map_scene)
                    self.first_time = False

                self.map_scene.addEllipse(x - int(self.diameter), y - int(self.diameter/2), 
                                      self.diameter, self.diameter,
                                      self.vehicle_pen)
                self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))


                points = [[x - self.C*math.sin(angle), y - self.C*math.cos(angle)], 
                          [x - math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                           y + math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2],
                          [x + math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                           y - math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2]]

                qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])
                self.map_scene.addPolygon(qpoly, QPen(Qt.blue), QBrush(Qt.blue)) 

    def stage_changed(self):
        self.repaint()

    def calculate_path(self):

        self.draw_stage()
        avg_result_dict, result_dict_list = self.dplanner.plan(args_train, args, self.rm2d_planner, self.test_env_number, self.start_point, self.goal_point)
        self.path = copy.copy(result_dict_list[0])

        for i in range(len(self.path)-1):
            x1 = int(self.path[i][0]/scale)
            y1 = int(map_size - self.path[i][1]/scale)
            x2 = int(self.path[i+1][0]/scale)
            y2 = int(map_size - self.path[i+1][1]/scale)
            self.map_scene.addLine(QLineF(x1, y1, x2, y2), self.path_line) 
            if(i == 0):
                self.map_scene.addEllipse(x1 - int(self.dot_size/2), y1 - int(self.dot_size/2),
                                          self.dot_size, self.dot_size, QPen(Qt.green), QBrush(Qt.green)) 
            elif(i == len(self.path)-2):
                self.map_scene.addEllipse(x2 - int(self.dot_size/2), y2 - int(self.dot_size/2),
                                          self.dot_size, self.dot_size, QPen(Qt.red), QBrush(Qt.red))
            else:
                self.map_scene.addEllipse(x2 - int(self.middle_dot_size/2), y2 - int(self.middle_dot_size/2),
                                          self.middle_dot_size, self.middle_dot_size, QPen(Qt.black), QBrush(Qt.black))
        
        pixmap2cv = self.ui.map_graphicsView.grab(QRect(QPoint(1,1),QSize(500, 500)))
        frame = self.pixmap_to_cv(pixmap2cv)
        frame = cv2.resize(frame, (500, 500))
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        cv2.imwrite(os.environ['HOME'] + '/diffusion_path_planning/src/robot_simulation/robot_gazebo/worlds/path_tracking_stage/meshes/diffusion_stage.png', frame)
        install_dir = Path(os.environ['HOME'] + '/diffusion_path_planning/install/robot_gazebo/share/robot_gazebo/worlds/path_tracking_stage/meshes')
        if install_dir.exists():
            cv2.imwrite(os.environ['HOME'] + '/diffusion_path_planning/install/robot_gazebo/share/robot_gazebo/worlds/path_tracking_stage/meshes/diffusion_stage.png', frame)

    def path_following(self):
        if(len(self.path) > 0):
            path_pub = Float64MultiArray(data=np.ravel(self.path))
            self.pub.publish(path_pub)
        else:
            print("There is no path have been generated!")

    def update(self):
        rclpy.spin_once(self.sub_node, timeout_sec=0)

    def repaint(self):
        self.draw_stage()
        self.map_scene.addEllipse(self.start_point[0]/scale - int(self.dot_size/2), (map_size - self.start_point[1]/scale)  - int(self.dot_size/2),
                                  self.dot_size, self.dot_size, QPen(Qt.green), QBrush(Qt.green)) 
        self.map_scene.addEllipse(self.goal_point[0]/scale - int(self.dot_size/2), (map_size - self.goal_point[1]/scale) - int(self.dot_size/2),
                                  self.dot_size, self.dot_size, QPen(Qt.red), QBrush(Qt.red))

    def pixmap_to_cv(self, pixmap):
        qimage = pixmap.toImage()
        w, h, d = qimage.size().width(), qimage.size().height(), qimage.depth()
        bytes_ = qimage.bits().asstring(w * h * d // 8)
        arr = np.frombuffer(bytes_, dtype=np.uint8).reshape((h, w, d // 8))
        im_bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return im_bgr

if __name__ == '__main__':
    ## training args
    args_train = Parser().parse_args('diffusion')
    args = Parser().parse_args('plan')
    loadpath = args.logbase, args.dataset, args_train.exp_name,
    latest_e = utils.get_latest_epoch(loadpath)
    n_e = round(latest_e // 1e5) + 1 # all
    start_e = 5e5; # 2e5 end_e = 
    depoch_list = np.arange(start_e, int(n_e * 1e5), int(1e5), dtype=np.int32).tolist()


    # args.horizon = # default 48
    args.repl_dn_steps = 3
    args.n_replan_trial = 3

    if 'Dyn' not in args.dataset:
        ## Static Maze2D
        args.seq_eval = False
        depoch = int(19e5)
        args.ddim_steps = 8

        args.n_prob_env = 1 #20
        args.use_ddim = True # False # True
        args.load_unseen_maze = True
        args.cond_w = 2.0 # 2.0
        args.do_replan = False # False # True
        args.n_vis = 20 if args.do_replan else 20
        args.vis_start_idx = 0
        args.samples_perprob = 10
        args.plan_n_maze = 1
    else:
        ## Dynamic rm2d
        args.seq_eval = False
        depoch = int(19e5)
        args.horizon = 48
        args.ddim_steps = 8
        
        args.n_prob_env = 20
        args.use_ddim = True
        args.load_unseen_maze = True
        args.cond_w = 2.0
        args.do_replan = False ## replan not supported yet
        args.n_vis = 0 # 2
        args.vis_start_idx = 0
        args.samples_perprob = 20
        args.plan_n_maze = 1


    sub_dir = f'{datetime.now().strftime("%y%m%d-%H%M%S")}-nm{int(args.plan_n_maze)}'
    args.savepath = osp.join(args.savepath, sub_dir)

    args_train.diffusion_epoch = depoch
    args.diffusion_epoch = depoch
        
    app = QApplication(sys.argv)
    window = Path_Visualization_Simulator()
    window.show()
    sys.exit(app.exec_())