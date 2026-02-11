#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf
import threading
import os
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class StartCloseMaster:
    def __init__(self):
        rospy.init_node('start_close_master')
        
        self.poses = {"uav0": PoseStamped(), "uav1": PoseStamped()}
        self.states = {"uav0": State(), "uav1": State()}
        self.gt_pos = {"uav0": None, "uav1": None}

        for ns in ["uav0", "uav1"]:
            rospy.Subscriber(f"/{ns}/mavros/state", State, lambda msg, n=ns: self.states.update({n: msg}))
            rospy.Subscriber(f"/{ns}/mavros/local_position/pose", PoseStamped, lambda msg, n=ns: self.poses.update({n: msg}))
            
        self.uav0_pub = rospy.Publisher("/uav0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.uav1_pub = rospy.Publisher("/uav1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gt_cb)

        self.uav0_mode = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)
        self.uav0_arm = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
        self.uav1_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
        self.uav1_arm = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)

    def gt_cb(self, msg):
        try:
            for ns in ["uav0", "uav1"]:
                if ns in msg.name:
                    idx = msg.name.index(ns)
                    self.gt_pos[ns] = msg.pose[idx].position
        except: pass

    def disable_failsafes(self):
        rospy.loginfo("--- 배터리 및 안전 장치 해제 중 ---")
        for ns in ["uav0", "uav1"]:
            os.system(f"rosrun mavros mavcmd -n {ns} param set BAT_LOW_THR 0.0 > /dev/null")
            os.system(f"rosrun mavros mavcmd -n {ns} param set COM_LOW_BAT_ACT 0 > /dev/null")

    def takeoff_sequence(self):
        rate = rospy.Rate(20)
        vel = TwistStamped()
        for _ in range(60):
            self.uav0_pub.publish(vel); self.uav1_pub.publish(vel); rate.sleep()

        while not rospy.is_shutdown():
            if self.states["uav0"].armed and self.states["uav1"].armed: break
            self.uav0_mode(custom_mode='OFFBOARD'); self.uav0_arm(True)
            self.uav1_mode(custom_mode='OFFBOARD'); self.uav1_arm(True)
            vel.twist.linear.z = 0.5
            self.uav0_pub.publish(vel); self.uav1_pub.publish(vel); rospy.sleep(0.5)
        rospy.loginfo("--- 이륙 완료! 코앞에서 비행을 시작합니다 ---")

    def observer_logic(self):
        """ uav0: 시선 분산 트래킹 """
        rate = rospy.Rate(30)
        last_offset_t = rospy.get_time()
        y_off, z_off = 0.0, 0.0

        while not rospy.is_shutdown():
            if self.gt_pos["uav0"] and self.gt_pos["uav1"]:
                if rospy.get_time() - last_offset_t > 2.0:
                    y_off = (np.random.rand() - 0.5) * 0.5 
                    z_off = (np.random.rand() - 0.5) * 2.5
                    last_offset_t = rospy.get_time()

                dx = self.gt_pos["uav1"].x - self.gt_pos["uav0"].x
                dy = self.gt_pos["uav1"].y - self.gt_pos["uav0"].y
                target_yaw = math.atan2(dy, dx) + y_off

                q = self.poses["uav0"].pose.orientation
                _, _, curr_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                
                yaw_err = target_yaw - curr_yaw
                while yaw_err > math.pi: yaw_err -= 2*math.pi
                while yaw_err < -math.pi: yaw_err += 2*math.pi

                vel = TwistStamped()
                vel.header.stamp = rospy.Time.now()
                vel.twist.linear.z = (5.0 + z_off - self.poses["uav0"].pose.position.z) * 0.7
                vel.twist.angular.z = yaw_err * 5.0 # 더 빠른 반응
                self.uav0_pub.publish(vel)
            rate.sleep()

    def target_logic(self):
        """ uav1: 시작하자마자 1.2m 근접 모드 """
        rate = rospy.Rate(20)
        start_t = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - start_t
            
            # [핵심 수정] -cos 함수를 써서 t=0일 때 거리가 1.2m가 되게 함
            # 주기(0.3)도 더 빠르게 높여서 금방 다가왔다 금방 멀어지게 함
            r = 5.6 - 4.4 * math.cos(0.3 * t)
            
            omega = 0.4 # 회전 속도도 더 시원시원하게 상향
            target_z = 5.5 + 2.0 * math.sin(0.3 * t) # 고도 주기 동일화
            
            vel = TwistStamped()
            vel.header.stamp = rospy.Time.now()
            vel.twist.linear.x = -r * omega * math.sin(omega * t)
            vel.twist.linear.y = r * omega * math.cos(omega * t)
            vel.twist.linear.z = (target_z - self.poses["uav1"].pose.position.z) * 0.8
            
            self.uav1_pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        master = StartCloseMaster()
        master.disable_failsafes()
        master.takeoff_sequence()
        threading.Thread(target=master.observer_logic).start()
        threading.Thread(target=master.target_logic).start()
        rospy.spin()
    except Exception as e: print(e)
