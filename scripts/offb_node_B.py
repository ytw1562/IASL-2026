#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class SimpleWaypointControl:
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # 웨이포인트 설정 [x, y, z]
        self.waypoints = [
            [0, 0, 5.5],
            [15, 0, 5.5],
            [15, 15, 5.5],
            [0, 15, 5.5],
            [0, 0, 5.5],
            [0, 7.5, 18.5],
            [0, 15, 5.5],
            [0, 0, 5.5]
        ]
        self.current_wp_idx = 0
        self.dist_threshold = 0.5  # 목표 지점 50cm 이내 도달 시 다음 지점으로

        # ROS 통신 설정
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def get_distance(self, wp):
        """현재 위치와 목표 지점 사이의 직선 거리 계산"""
        return math.sqrt(
            (wp[0] - self.current_pose.pose.position.x)**2 +
            (wp[1] - self.current_pose.pose.position.y)**2 +
            (wp[2] - self.current_pose.pose.position.z)**2
        )

    def get_yaw_to_target(self, wp):
        """진행 방향을 바라보기 위한 Yaw 각도 계산"""
        dy = wp[1] - self.current_pose.pose.position.y
        dx = wp[0] - self.current_pose.pose.position.x
        return math.atan2(dy, dx)

    def euler_to_quaternion(self, yaw):
        """Yaw 각도를 MAVROS용 쿼터니언으로 변환"""
        return [0, 0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def run(self):
        rate = rospy.Rate(20) # 20Hz

        # FCU 연결 대기
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()

        # 이륙 지점을 초기 Setpoint로 설정
        target_pose = PoseStamped()
        target_pose.pose.position.x = self.waypoints[0][0]
        target_pose.pose.position.y = self.waypoints[0][1]
        target_pose.pose.position.z = self.waypoints[0][2]

        # 시작 전 Setpoint 미리 보내기 (Offboard 수락 조건)
        for _ in range(100):
            if rospy.is_shutdown(): break
            self.local_pos_pub.publish(target_pose)
            rate.sleep()

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            # 모드 및 시동 관리
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
                self.set_mode_client.call(SetModeRequest(custom_mode='OFFBOARD'))
                last_req = rospy.Time.now()
            elif not self.current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
                self.arming_client.call(CommandBoolRequest(value=True))
                last_req = rospy.Time.now()

            # --- 메인 이동 로직 ---
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                target_wp = self.waypoints[self.current_wp_idx]
                
                # 1. 목표 지점 좌표 그대로 전송 (속도 조절 없음)
                target_pose.pose.position.x = target_wp[0]
                target_pose.pose.position.y = target_wp[1]
                target_pose.pose.position.z = target_wp[2]

                # 2. 진행 방향에 맞춰 Yaw 회전
                # (목표 지점과 거리가 어느 정도 있을 때만 회전 계산)
                if self.get_distance(target_wp) > 0.3:
                    yaw = self.get_yaw_to_target(target_wp)
                    q = self.euler_to_quaternion(yaw)
                    target_pose.pose.orientation.z = q[2]
                    target_pose.pose.orientation.w = q[3]

                # 3. 목표 도달 판정 및 다음 지점 갱신
                if self.get_distance(target_wp) < self.dist_threshold:
                    self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
                    rospy.loginfo(f"목표 도달: {target_wp} -> 다음으로 이동")

            self.local_pos_pub.publish(target_pose)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("simple_wp_node")
    try:
        controller = SimpleWaypointControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass