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
        
        # ROS 통신 설정
        rospy.Subscriber(f"/{self.ns}/mavros/state", State, self.state_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.vel_pub = rospy.Publisher(f"/{self.ns}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        self.arming_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy(f'/{self.ns}/mavros/set_mode', SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def ready_to_fly(self):
        rospy.loginfo(f"[{self.ns}] 연결 대기 및 안정화 중...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        rospy.sleep(10.0) # 안정화 시간
        
        rate = rospy.Rate(20)
        for _ in range(100):
            self.vel_pub.publish(self.target_vel)
            rate.sleep()
            
        while not rospy.is_shutdown() and not self.current_state.armed:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode_srv(custom_mode='OFFBOARD')
            self.arming_srv(True)
            rospy.sleep(1.0)
        rospy.loginfo(f"[{self.ns}] 이륙 시작!")

    def run(self):
        self.ready_to_fly()
        rate = rospy.Rate(20)
        start_time = rospy.get_time()
        radius, omega = 5.0, 0.15 
        
        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time
            z_pos = self.current_pose.pose.position.z
            
            self.target_vel.header.stamp = rospy.Time.now()
            self.target_vel.header.frame_id = "map"
            
            # 고도 유지 제어
            self.target_vel.twist.linear.z = (5.0 - z_pos) * 0.6
            
            # 원운동 궤적 계산
            if z_pos > 4.5:
                self.target_vel.twist.linear.x = -radius * omega * math.sin(omega * t)
                self.target_vel.twist.linear.y = radius * omega * math.cos(omega * t)
                
            self.vel_pub.publish(self.target_vel)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('target_drone_node')
    drone = TargetDrone("uav1")
    drone.run()
