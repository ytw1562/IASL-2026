#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class TargetDrone:
    def __init__(self, ns="uav1"):
        self.ns = ns
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_vel = TwistStamped()
        
        # ROS 통신
        rospy.Subscriber(f"/{self.ns}/mavros/state", State, self.state_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.vel_pub = rospy.Publisher(f"/{self.ns}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.arming_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/set_mode', SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def run(self):
        # 1. 연결 대기 (무한 루프)
        rate = rospy.Rate(20)
        rospy.loginfo(f"[{self.ns}] MAVROS 연결 대기 중...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo(f"[{self.ns}] 연결 성공!")

        # 2. 오프보드 신호 미리 전송 (안전장치 해제용)
        for _ in range(100):
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

        # 3. 모드 전환 및 시동 (타임아웃 없이 계속 시도)
        rospy.loginfo(f"[{self.ns}] 시동 시도 중...")
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD":
                try: self.set_mode_srv(custom_mode='OFFBOARD')
                except: pass
            
            if not self.current_state.armed:
                try: self.arming_srv(True)
                except: pass
            
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                rospy.loginfo(f"[{self.ns}] 이륙 성공! 원운동 시작.")
                break
            
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

        # 4. 원운동 비행
        start_time = rospy.get_time()
        radius, omega = 5.0, 0.2
        
        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time
            z = self.current_pose.pose.position.z
            
            # 고도 5m 유지
            self.target_vel.twist.linear.z = (5.0 - z) * 0.5
            
            # 고도가 어느 정도 올라가면 회전 시작
            if z > 4.0:
                self.target_vel.twist.linear.x = -radius * omega * math.sin(omega * t)
                self.target_vel.twist.linear.y = radius * omega * math.cos(omega * t)
                self.target_vel.twist.angular.z = omega # 진행 방향 보기
            
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('target_drone_node', anonymous=True)
        # 런치 파일 네임스페이스 자동 감지
        my_ns = rospy.get_namespace().strip('/')
        if not my_ns: my_ns = "uav1"
        
        TargetDrone(ns=my_ns).run()
    except rospy.ROSInterruptException: pass
