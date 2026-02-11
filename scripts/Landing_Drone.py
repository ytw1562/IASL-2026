#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge

class KAUHighAltitudeNoSmooth:
    def __init__(self):
        rospy.init_node('kau_high_altitude_no_smooth', anonymous=True)
        self.bridge = CvBridge()

        # HSV 색상 (노란색)
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([35, 255, 255])

        # 제어 파라미터
        self.base_kp = 0.005    
        self.base_kd = 0.010    
        
        # [삭제됨] 스무딩 팩터 제거
        # self.SMOOTH_FACTOR = 0.8 
        
        self.DEAD_ZONE = 15

        # --- [튜닝된 파라미터] ---
        self.BLIND_DURATION = 3.0   # 관성 주행 시간 (3초)
        self.BLIND_ENTRY_Y = 180    
        self.LAND_TRIGGER_ALT = 0.65 
        self.LAND_ERR_PIXEL = 60
        self.SEARCH_ALT = 4       # 탐색 고도 (4m)
        self.FORCE_LAND_ALT = 0.75
        
        self.is_takeoff_finished = False
        
        # 변수 초기화
        self.mem_vx = 0.0
        self.mem_vy = 0.0
        # [삭제됨] 스무딩용 변수 제거
        # self.smooth_vx = 0.0
        # self.smooth_vy = 0.0

        self.last_seen_time = rospy.get_time()
        self.last_front_y = 0.0 
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0

        self.front_found = False
        self.down_mode = "NONE" 
        self.front_err_x = 0
        self.down_err_x = 0
        self.down_err_y = 0
        
        self.front_frame = np.zeros((240, 320, 3), np.uint8)
        self.down_frame = np.zeros((240, 320, 3), np.uint8)

        # ArUco 설정
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dict, cv2.aruco.DetectorParameters())

        # ROS 통신
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.front_cb)
        rospy.Subscriber('/iris/down_cam/image_raw', Image, self.down_cb)
        
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        self.curr_state = State()
        self.curr_pose = PoseStamped()

    def state_cb(self, msg): self.curr_state = msg
    def pose_cb(self, msg): self.curr_pose = msg
    def get_yaw(self):
        q = self.curr_pose.pose.orientation
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    # --- 전방 카메라 ---
    def front_cb(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            small = cv2.resize(frame, (320, 240))
            hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            self.front_found = False
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(c) > 200:
                    x, y, w, h = cv2.boundingRect(c)
                    self.front_err_x = (x + w/2) - 160
                    self.front_found = True
                    self.last_front_y = y + h/2
                    self.last_seen_time = rospy.get_time()
                    cv2.rectangle(small, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.line(small, (0, self.BLIND_ENTRY_Y), (320, self.BLIND_ENTRY_Y), (255, 0, 0), 1)
            cv2.putText(small, "BLIND LIMIT", (5, self.BLIND_ENTRY_Y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            self.front_frame = small
        except: pass

    # --- 하방 카메라 ---
    def down_cb(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = frame.shape[:2]
            center_x = int(w / 2)
            center_y = int(h / 2)

            corners, ids, _ = self.detector.detectMarkers(frame)
            found_aruco = False
            
            # [디버깅 UI]
            id_str = "NONE"
            if ids is not None:
                id_str = str(ids.flatten().tolist())
            cv2.putText(frame, f"VISIBLE IDS: {id_str}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            if ids is not None:
                tgt_idx = -1
                
                # ★ 1순위: 작은 마커 (ID 1)
                if 1 in ids: 
                    tgt_idx = np.where(ids == 1)[0][0]
                    self.down_mode = "ARUCO_SMALL"
                
                # ★ 2순위: 큰 마커 (ID 0)
                elif 0 in ids: 
                    tgt_idx = np.where(ids == 0)[0][0]
                    self.down_mode = "ARUCO_LARGE"
                
                if tgt_idx != -1:
                    cx = np.mean(corners[tgt_idx][0][:, 0])
                    cy = np.mean(corners[tgt_idx][0][:, 1])
                    
                    self.down_err_x = cx - center_x
                    self.down_err_y = cy - center_y
                    
                    found_aruco = True
                    self.last_seen_time = rospy.get_time() 
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 마커 없으면 색상 인식
            if not found_aruco:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                found_color = False
                if cnts:
                    c = max(cnts, key=cv2.contourArea)
                    if cv2.contourArea(c) > 500:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            self.down_err_x = cx - center_x
                            self.down_err_y = cy - center_y
                            self.down_mode = "COLOR" 
                            found_color = True
                            self.last_seen_time = rospy.get_time()
                            x, y, w, h = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                if not found_color: self.down_mode = "NONE"

            cv2.circle(frame, (center_x, center_y), self.LAND_ERR_PIXEL, (0, 0, 255), 2)
            cv2.putText(frame, "LANDING ZONE", (center_x - 60, center_y + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.down_frame = cv2.resize(frame, (320, 240))
        except: pass

    def run(self):
        rate = rospy.Rate(30)
        rospy.loginfo("High Altitude System Ready (NO SMOOTHING).")
        while not rospy.is_shutdown() and not self.curr_state.connected: rate.sleep()
        for _ in range(50): self.vel_pub.publish(Twist()); rate.sleep()

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            time_since_seen = current_time - self.last_seen_time
            
            if self.front_frame is not None and self.down_frame is not None:
                combined = np.hstack((self.front_frame, self.down_frame))
                status_text = "SEARCHING..."
                color = (255, 255, 255)

                if self.down_mode == "ARUCO_SMALL":
                    status_text = "TARGET: [SMALL] (CENTER)"
                    color = (0, 255, 0)
                elif self.down_mode == "ARUCO_LARGE":
                    status_text = "TARGET: [LARGE] (APPROACH)"
                    color = (255, 165, 0)
                elif self.down_mode == "COLOR":
                    status_text = "TARGET: [YELLOW BOX]"
                    color = (0, 255, 255)
                elif self.front_found:
                    status_text = "APPROACHING..."
                    color = (255, 0, 0)
                elif time_since_seen < self.BLIND_DURATION and self.last_front_y > self.BLIND_ENTRY_Y:
                    status_text = "PREDICTION MODE"
                    color = (0, 255, 255) 
                
                cv2.putText(combined, status_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.imshow("KAU Flight Monitor", combined)
                cv2.waitKey(1)

            if self.curr_state.mode != "OFFBOARD": self.set_mode_srv(custom_mode='OFFBOARD')
            elif not self.curr_state.armed: self.arming_srv(value=True)

            z = self.curr_pose.pose.position.z
            yaw = self.get_yaw()
            cmd = Twist()
            
            # [Step 0] 독수리 이륙 (3.5m까지 상승)
            if not self.is_takeoff_finished:
                if z < self.SEARCH_ALT - 0.2:
                    cmd.linear.z = 0.6  
                    self.vel_pub.publish(cmd)
                    rate.sleep()
                    continue 
                else:
                    self.is_takeoff_finished = True
                    rospy.loginfo("HUNTING START (High Altitude).")

            # [Step 1] 하방 추적
            if self.down_mode != "NONE":
                adaptive_ratio = np.clip(z / 1.5, 0.3, 1.0)
                if "COLOR" in self.down_mode: adaptive_ratio *= 0.8

                current_kp = self.base_kp * adaptive_ratio
                current_kd = self.base_kd * adaptive_ratio

                p_term_x = self.down_err_y * current_kp
                p_term_y = self.down_err_x * current_kp
                d_term_x = (self.down_err_y - self.prev_err_y) * current_kd
                d_term_y = (self.down_err_x - self.prev_err_x) * current_kd

                target_bx = -(p_term_x + d_term_x)
                target_by = -(p_term_y + d_term_y)

                dist = np.sqrt(self.down_err_x**2 + self.down_err_y**2)
                if dist < self.DEAD_ZONE:
                    target_bx = 0.0
                    target_by = 0.0

                if z < 0.5:
                    target_bx = np.clip(target_bx, -0.1, 0.1)
                    target_by = np.clip(target_by, -0.1, 0.1)

                # [수정됨] 스무딩 제거 -> 계산된 값 바로 사용
                # self.smooth_vx = (self.smooth_vx * self.SMOOTH_FACTOR) + (target_bx * (1 - self.SMOOTH_FACTOR))
                # self.smooth_vy = (self.smooth_vy * self.SMOOTH_FACTOR) + (target_by * (1 - self.SMOOTH_FACTOR))
                
                self.prev_err_x = self.down_err_x
                self.prev_err_y = self.down_err_y
                
                # 관성 주행용으로 현재 속도 저장 (스무딩 값 아님)
                self.mem_vx = target_bx
                self.mem_vy = target_by
                
                # 하강 속도 결정 (높은 고도에서 중심 잡으면 내려옴)
                if self.down_mode == "ARUCO_SMALL":
                    if dist < 40: cmd.linear.z = -0.2
                    else: cmd.linear.z = -0.1
                else:
                    if dist < 60: cmd.linear.z = -0.3   
                    elif dist < 150: cmd.linear.z = -0.15 
                    else: cmd.linear.z = 0.0
                
                # 강제 착륙
                if (z < self.LAND_TRIGGER_ALT and dist < self.LAND_ERR_PIXEL) or (z < self.FORCE_LAND_ALT):
                    rospy.logwarn(f"LANDING TRIGGERED! (Alt: {z:.2f}m)")
                    self.set_mode_srv(custom_mode='AUTO.LAND')
                    break
                
                # [수정됨] 스무딩된 값 대신 원본 값 사용
                # target_bx = self.smooth_vx
                # target_by = self.smooth_vy

            # [Step 2] 전방 추적 (높은 고도 3.5m 유지)
            elif self.front_found:
                cmd.angular.z = -self.front_err_x * 0.005
                target_bx = 0.8
                cmd.linear.z = 0.6 * (self.SEARCH_ALT - z) # 3.5m 유지
                # [수정됨] 스무딩 제거
                # self.smooth_vx = 0.8
                # self.smooth_vy = 0.0
                self.mem_vx = 0.8 
                self.mem_vy = 0.0
                
            # [Step 3] 예측 주행 (3초 동안 지속)
            elif time_since_seen < self.BLIND_DURATION and self.last_front_y > self.BLIND_ENTRY_Y:
                target_bx = self.mem_vx
                target_by = self.mem_vy
                # [수정됨] 스무딩 변수 업데이트 제거
                # self.smooth_vx = target_bx 
                # self.smooth_vy = target_by
                self.prev_err_x = 0
                self.prev_err_y = 0
                cmd.linear.z = 0.0

            # [Step 4] 탐색 (3.5m 유지)
            else:
                if z < self.SEARCH_ALT: cmd.linear.z = 0.4
                elif z > self.SEARCH_ALT + 0.5: cmd.linear.z = -0.4
                else: cmd.linear.z = 0.0
                cmd.angular.z = -0.4 
                target_bx = 0.0
                target_by = 0.0
                # [수정됨] 스무딩 변수 초기화 제거
                # self.smooth_vx = 0.0
                # self.smooth_vy = 0.0

            cmd.linear.x = target_bx * np.cos(yaw) - target_by * np.sin(yaw)
            cmd.linear.y = target_bx * np.sin(yaw) + target_by * np.cos(yaw)
            cmd.linear.x = np.clip(cmd.linear.x, -1.0, 1.0)
            cmd.linear.y = np.clip(cmd.linear.y, -1.0, 1.0)
            self.vel_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try: KAUHighAltitudeNoSmooth().run()
    except: pass
