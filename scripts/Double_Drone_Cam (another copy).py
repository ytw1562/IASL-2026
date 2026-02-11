#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge
from ultralytics import YOLO

class FollowerDrone:
    def __init__(self, ns="uav0"):
        self.ns = ns
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_vel = TwistStamped()
        
        # 제어 데이터
        self.target_found = False
        self.center_error = 0
        self.target_area = 0
        self.is_processing = False

        # 비례 제어 게인 및 설정값
        self.yaw_kp = 0.003
        self.dist_kp = 0.0002
        self.area_setpoint = 4500

        # ROS 통신
        rospy.Subscriber(f"/{self.ns}/mavros/state", State, self.state_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber(f"/{self.ns}/usb_cam/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.vel_pub = rospy.Publisher(f"/{self.ns}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.arming_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/set_mode', SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def get_current_yaw(self):
        q = self.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        if self.is_processing: return
        try:
            self.is_processing = True
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            small_img = cv2.resize(cv_img, (320, 240))
            h, w = small_img.shape[:2]
            
            results = self.model.predict(small_img, conf=0.25, verbose=False)
            self.target_found = len(results[0].boxes) > 0
            
            if self.target_found:
                box = results[0].boxes[0]
                x1, y1, x2, y2 = box.xyxy[0]
                cx = (x1 + x2) / 2
                self.center_error = cx - (w / 2)
                self.target_area = (x2 - x1) * (y2 - y1)
                
                plot_img = results[0].plot()
                cv2.imshow(f"{self.ns} View", plot_img)
            else:
                cv2.imshow(f"{self.ns} View", small_img)
            cv2.waitKey(1)
            self.is_processing = False
        except: self.is_processing = False

    def ready_to_fly(self):
        rospy.loginfo(f"[{self.ns}] 시스템 안정화 대기...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        rospy.sleep(15.0)
        
        rate = rospy.Rate(20)
        for _ in range(100):
            self.vel_pub.publish(self.target_vel)
            rate.sleep()
            
        while not rospy.is_shutdown() and not self.current_state.armed:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode_srv(custom_mode='OFFBOARD')
            self.arming_srv(True)
            rospy.sleep(1.0)
        rospy.loginfo(f"[{self.ns}] 추격 시작!")

    def run(self):
        self.ready_to_fly()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            z_pos = self.current_pose.pose.position.z
            curr_yaw = self.get_current_yaw()
            
            self.target_vel.header.stamp = rospy.Time.now()
            self.target_vel.header.frame_id = "map"
            self.target_vel.twist.linear.z = (5.0 - z_pos) * 0.7

            if self.target_found:
                # 거리 비례 제어: $v_x = (Area_{set} - Area_{curr}) \times K_{dist}$
                area_error = self.area_setpoint - self.target_area
                body_x = np.clip(area_error * self.dist_kp, -0.6, 1.0)
                
                # 좌표 변환 (Body -> Map)
                self.target_vel.twist.linear.x = body_x * math.cos(curr_yaw)
                self.target_vel.twist.linear.y = body_x * math.sin(curr_yaw)
                
                # 회전 비례 제어: $\omega_z = -Error_{center} \times K_{yaw}$
                self.target_vel.twist.angular.z = -self.yaw_kp * self.center_error
            else:
                self.target_vel.twist.linear.x = 0.0
                self.target_vel.twist.linear.y = 0.0
                self.target_vel.twist.angular.z = 0.3
            
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('follower_drone_node')
    drone = FollowerDrone("uav0")
    drone.run()
