#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion

# --- [클래스 1] MAVROS 통신 인터페이스 ---
class MavrosInterface:
    def __init__(self, ns):
        self.ns = ns
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # 네임스페이스가 적용된 토픽 구독 및 발행
        rospy.Subscriber(f"{ns}/mavros/state", State, self.state_cb)
        rospy.Subscriber(f"{ns}/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.vel_pub = rospy.Publisher(f"{ns}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        self.arming_client = rospy.ServiceProxy(f"{ns}/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f"{ns}/mavros/set_mode", SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg
    def get_yaw(self):
        q = self.current_pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        return yaw

# --- [클래스 2] PID 드론 컨트롤러 ---
class PIDDroneController:
    def __init__(self, ns):
        self.mav = MavrosInterface(ns)
        
        # [고정 게인 설정]
        self.kp, self.ki, self.kd = 1.2, 0.02, 0.2
        self.kp_y, self.ki_y, self.kd_y = 2.0, 0.01, 0.1
        
        self.int_x = self.int_y = self.int_z = self.int_yaw = 0
        self.pre_x = self.pre_y = self.pre_z = self.pre_yaw = 0

    def compute_and_publish(self, dt, target):
        if not self.mav.current_state.connected: return 999
        
        curr_p = self.mav.current_pose.pose.position
        curr_y = self.mav.get_yaw()

        # PID 공식: $V = K_p \cdot e + K_i \int e \, dt + K_d \frac{de}{dt}$
        ex, ey, ez = target[0] - curr_p.x, target[1] - curr_p.y, target[2] - curr_p.z
        
        self.int_x += ex * dt; vx = self.kp * ex + self.ki * self.int_x + self.kd * (ex - self.pre_x) / dt
        self.int_y += ey * dt; vy = self.kp * ey + self.ki * self.int_y + self.kd * (ey - self.pre_y) / dt
        self.int_z += ez * dt; vz = self.kp * ez + self.ki * self.int_z + self.kd * (ez - self.pre_z) / dt
        self.pre_x, self.pre_y, self.pre_z = ex, ey, ez

        # Yaw PID
        target_yaw = math.atan2(ey, ex)
        e_yaw = target_yaw - curr_y
        while e_yaw > math.pi: e_yaw -= 2*math.pi
        while e_yaw < -math.pi: e_yaw += 2*math.pi
        self.int_yaw += e_yaw * dt
        vyaw = self.kp_y * e_yaw + self.ki_y * self.int_yaw + self.kd_y * (e_yaw - self.pre_yaw) / dt
        self.pre_yaw = e_yaw

        # 속도 제한 (Saturation)
        max_v = 2.0
        vx = max(min(vx, max_v), -max_v)
        vy = max(min(vy, max_v), -max_v)

        # 명령 발행
        vel = TwistStamped()
        vel.header.stamp = rospy.Time.now()
        vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z = vx, vy, vz
        vel.twist.angular.z = vyaw
        self.mav.vel_pub.publish(vel)
        
        return math.sqrt(ex**2 + ey**2 + ez**2)

# --- [메인 실행부] ---
def main():
    rospy.init_node("multi_drone_patrol")
    
    # 드론 2대 생성
    uav0 = PIDDroneController("uav0")
    uav1 = PIDDroneController("uav1")
    
    # 순찰 경로 (고도를 다르게 설정하여 충돌 방지)
    path0 = [[0, 0, 5], [10, 0, 5], [10, 10, 5], [0, 10, 5]]
    path1 = [[0, 2, 8], [10, 2, 8], [10, 12, 8], [0, 12, 8]]
    
    idx0 = idx1 = 0
    rate = rospy.Rate(20)
    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        
        # 각 드론의 상태 관리 (Offboard & Arm)
        for drone in [uav0, uav1]:
            if drone.mav.current_state.mode != "OFFBOARD" and (now - last_req > rospy.Duration(5.0)):
                drone.mav.set_mode_client.call(SetModeRequest(custom_mode='OFFBOARD'))
            elif not drone.mav.current_state.armed and (now - last_req > rospy.Duration(5.0)):
                drone.mav.arming_client.call(CommandBoolRequest(value=True))
        
        if now - last_req > rospy.Duration(5.0): last_req = now

        # 비행 및 미션 수행
        dist0 = uav0.compute_and_publish(0.05, path0[idx0])
        dist1 = uav1.compute_and_publish(0.05, path1[idx1])

        # 도착 판정 및 무한 반복 (%)
        if dist0 < 1.5: idx0 = (idx0 + 1) % len(path0)
        if dist1 < 1.8: idx1 = (idx1 + 1) % len(path1)

        rate.sleep()

if __name__ == "__main__":
    main()