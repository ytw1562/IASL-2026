#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import cv2
import threading
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge
from ultralytics import YOLO

class DroneController(object):
    def __init__(self, namespace):
        self.ns = namespace
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_vel = TwistStamped()
        
        rospy.Subscriber(f"/{self.ns}/mavros/state", State, self.state_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/local_position/pose", PoseStamped, self.pose_cb)
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

    def ready_to_fly(self):
        rospy.loginfo(f"[{self.ns}] 연결 확인 중...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        rospy.loginfo(f"[{self.ns}] 시스템 안정화 대기 (15초)...")
        rospy.sleep(15.0)
        rate = rospy.Rate(20)
        for _ in range(100):
            self.vel_pub.publish(self.target_vel)
            rate.sleep()
        while not rospy.is_shutdown() and not self.current_state.armed:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode_srv(custom_mode='OFFBOARD')
            self.arming_srv(True)
            rospy.loginfo_throttle(2, f"[{self.ns}] 시동 대기 중...")
            rospy.sleep(1.0)
        rospy.loginfo(f"[{self.ns}] 이륙 준비 완료!")

class TargetDrone(DroneController): # UAV1 (도망 드론)
    def run(self):
        self.ready_to_fly()
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        radius, omega = 5.0, 0.15 
        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time
            z_pos = self.current_pose.pose.position.z
            self.target_vel.header.stamp = rospy.Time.now()
            self.target_vel.header.frame_id = "map"
            self.target_vel.twist.linear.z = (5.0 - z_pos) * 0.6
            if z_pos > 4.5:
                self.target_vel.twist.linear.x = -radius * omega * math.sin(omega * t)
                self.target_vel.twist.linear.y = radius * omega * math.cos(omega * t)
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

class FollowerDrone(DroneController): # UAV0 (회전/거리 모두 비례제어)
    def __init__(self, namespace):
        super(FollowerDrone, self).__init__(namespace)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.target_found = False
        self.center_error = 0
        self.target_area = 0
        self.is_processing = False

        # [제어 파라미터 최적화]
        self.yaw_kp = 0.003       # 회전 비례 상수
        self.dist_kp = 0.0002     # 거리 비례 상수 (작게 시작)
        self.area_setpoint = 4500 # 드론이 멈춰있길 원하는 면적 (320x240 기준)
        
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
                
                # 가이드라인(5등분)과 조준점 시각화
                plot_img = results[0].plot()
                for i in range(1, 5):
                    cv2.line(plot_img, (i * (w // 5), 0), (i * (w // 5), h), (255, 255, 255), 1)
                cv2.circle(plot_img, (int(cx), int((y1+y2)/2)), 5, (0, 255, 0), -1)
                cv2.imshow("Follower(uav0) Proportional Control", plot_img)
            else:
                cv2.imshow("Follower(uav0) Proportional Control", small_img)
            cv2.waitKey(1)
            self.is_processing = False
        except: self.is_processing = False

    def run(self):
        rospy.Subscriber(f"/{self.ns}/usb_cam/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.ready_to_fly()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            z_pos = self.current_pose.pose.position.z
            curr_yaw = self.get_current_yaw()
            
            self.target_vel.header.stamp = rospy.Time.now()
            self.target_vel.header.frame_id = "map"
            self.target_vel.twist.linear.z = (5.0 - z_pos) * 0.7

            if self.target_found:
                # 1. 거리 비례 제어 (Distance P-Control)
                # 오차 = 목표 면적 - 현재 면적 (멀수록 오차가 커짐)
                area_error = self.area_setpoint - self.target_area
                body_x = area_error * self.dist_kp
                
                # 안전을 위한 최대 전진/후진 속도 제한 (클리핑)
                body_x = np.clip(body_x, -0.6, 1.0)
                
                # 2. 기체 기준 속도를 월드 좌표계(ENU)로 변환
                self.target_vel.twist.linear.x = body_x * math.cos(curr_yaw)
                self.target_vel.twist.linear.y = body_x * math.sin(curr_yaw)
                
                # 3. 회전 비례 제어 (Yaw P-Control)
                self.target_vel.twist.angular.z = -self.yaw_kp * self.center_error
                
                rospy.loginfo_throttle(1, f"[uav0] AreaErr: {int(area_error)} -> SpeedX: {body_x:.2f}")
            else:
                self.target_vel.twist.linear.x = 0.0
                self.target_vel.twist.linear.y = 0.0
                self.target_vel.twist.angular.z = 0.3 # 수색 회전
            
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('full_proportional_manager')
        u1, u0 = TargetDrone("uav1"), FollowerDrone("uav0")
        threading.Thread(target=u1.run).start()
        threading.Thread(target=u0.run).start()
        rospy.spin()
    except: pass
    finally: cv2.destroyAllWindows()
