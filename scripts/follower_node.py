#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import cv2
import threading
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge
from ultralytics import YOLO

class FollowerDrone:
    def __init__(self, ns="uav0"):
        self.ns = ns
        self.bridge = CvBridge()
        
        # 모델 로드
        try:
            self.model = YOLO('yolo26m.pt')
            import torch
            if torch.cuda.is_available():
                self.model.to('cuda')
        except: pass

        self.lock = threading.Lock()
        self.target_vel = TwistStamped()
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        self.shared_found = False
        self.shared_error = 0
        self.shared_area = 0
        self.shared_v_error = 0
        self.is_processing = False
        self.prev_time = time.time()
        self.fps = 0.0

        # 게인
        self.yaw_kp = 0.003
        self.dist_kp = 0.0002
        self.area_setpoint = 4500 

        # [중요] 토픽 이름 명확화
        # 1. State 토픽
        state_topic = f"/{self.ns}/mavros/state"
        # 2. Pose 토픽
        pose_topic = f"/{self.ns}/mavros/local_position/pose"
        # 3. 이미지 토픽
        img_topic = f"/{self.ns}/usb_cam/image_raw"
        # 4. 명령 토픽 (가장 중요!)
        cmd_topic = f"/{self.ns}/mavros/setpoint_velocity/cmd_vel"

        rospy.loginfo(f"[{self.ns}] 구독 중인 토픽: {state_topic}")
        rospy.loginfo(f"[{self.ns}] 발행 예정 토픽: {cmd_topic}")

        rospy.Subscriber(state_topic, State, self.state_cb)
        rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)
        rospy.Subscriber(img_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        self.vel_pub = rospy.Publisher(cmd_topic, TwistStamped, queue_size=10)
        
        # 서비스
        self.arming_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/set_mode', SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def image_callback(self, msg):
        if self.is_processing: return
        try:
            self.is_processing = True
            current_time = time.time()
            dt = current_time - self.prev_time
            if dt > 0: self.fps = 1.0 / dt
            self.prev_time = current_time

            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model.predict(cv_img, conf=0.15, imgsz=640, verbose=False)
            
            found = len(results[0].boxes) > 0
            with self.lock:
                if found:
                    box = results[0].boxes[0]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
                    h, w = cv_img.shape[:2]
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    
                    self.shared_found = True
                    self.shared_error = float(cx - (w / 2))
                    self.shared_area = float((x2 - x1) * (y2 - y1))
                    self.shared_v_error = float((h / 2) - cy)
                else:
                    self.shared_found = False
            
            display_img = results[0].plot()
            fps_text = f"FPS: {self.fps:.1f}"
            cv2.putText(display_img, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(display_img, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(f"Follower ({self.ns})", cv2.resize(display_img, (480, 360)))
            cv2.waitKey(1)
            self.is_processing = False
        except: self.is_processing = False

    def get_current_yaw(self):
        q = self.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def ready_to_fly(self):
        rospy.loginfo(f"[{self.ns}] MAVROS 연결 대기 중...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        
        rospy.loginfo(f"[{self.ns}] 연결됨! OFFBOARD 신호 전송 시작...")
        
        # [핵심] OFFBOARD 진입을 위해 100번 미리 쏴줌
        rate = rospy.Rate(20)
        self.target_vel.twist.linear.z = 0.0 # 초기엔 가만히
        for _ in range(100):
            self.vel_pub.publish(self.target_vel)
            rate.sleep()
            
        rospy.loginfo(f"[{self.ns}] 모드 전환 및 시동 시도...")
        last_req = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                resp = self.set_mode_srv(custom_mode='OFFBOARD')
                if resp.mode_sent: rospy.loginfo(f"[{self.ns}] Offboard enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    resp = self.arming_srv(True)
                    if resp.success: rospy.loginfo(f"[{self.ns}] Vehicle armed")
                    last_req = rospy.Time.now()
            
            # 시동 걸리고 모드 바뀌면 루프 탈출
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                rospy.loginfo(f"[{self.ns}] 비행 준비 완료!")
                break
                
            # 대기 중에도 계속 데이터를 쏴줘야 안 튕김
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

    def run(self):
        self.ready_to_fly()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.lock:
                found = self.shared_found
                error = self.shared_error
                area = self.shared_area
                v_err = self.shared_v_error

            yaw = self.get_current_yaw()
            z_pos = self.current_pose.pose.position.z
            
            self.target_vel.header.stamp = rospy.Time.now()
            self.target_vel.header.frame_id = "map"
            
            # 고도 제어 (2m 유지)
            self.target_vel.twist.linear.z = (2.0 - z_pos) * 0.5

            if found:
                area_err = self.area_setpoint - area
                body_x = np.clip(area_err * self.dist_kp, -0.6, 1.2)
                self.target_vel.twist.linear.x = body_x * math.cos(yaw)
                self.target_vel.twist.linear.y = body_x * math.sin(yaw)
                self.target_vel.twist.linear.z = v_err * 0.005 # 상하 추적 추가
                self.target_vel.twist.angular.z = -self.yaw_kp * error
            else:
                self.target_vel.twist.linear.x = 0.0
                self.target_vel.twist.linear.y = 0.0
                self.target_vel.twist.angular.z = 0.0 # 탐색 중 회전 멈춤 (디버깅용)
            
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('follower_node', anonymous=True)
        my_ns = rospy.get_namespace().strip('/')
        if not my_ns: my_ns = "uav0"
        
        FollowerDrone(ns=my_ns).run()
    except rospy.ROSInterruptException: pass
