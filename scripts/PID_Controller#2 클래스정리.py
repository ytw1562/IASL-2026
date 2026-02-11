#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion

# --- [클래스 1] MAVROS 통신 인터페이스 (우체국 역할) ---
class MavrosInterface:
    def __init__(self):
        # 상태 및 위치 데이터 저장 변수
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # 구독(Subscribers)
        rospy.Subscriber("mavros/state", State, self.state_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        # 발행(Publishers)
        self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        # 서비스 클라이언트(Services)
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def get_yaw(self):
        """쿼터니언을 오일러 Yaw로 변환"""
        q = self.current_pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        return yaw

# --- [클래스 2] 드론 미션 컨트롤러 (두뇌 역할) ---
class DroneMission:
    def __init__(self):
        # 통신 인터페이스 객체 생성
        self.mav = MavrosInterface()
        
        # [게인 값 고정 설정]
        self.kp, self.ki, self.kd = 1.2, 0.02, 0.3
        self.kp_y, self.ki_y, self.kd_y = 1.5, 0.01, 0.1
        
        # 오차 계산용 변수들
        self.int_x = self.int_y = self.int_z = self.int_yaw = 0
        self.pre_x = self.pre_y = self.pre_z = self.pre_yaw = 0

    def compute_pid_output(self, dt, target):
        """PID 계산 로직만 담당"""
        curr_p = self.mav.current_pose.pose.position
        curr_y = self.mav.get_yaw()

        # 1. 위치 오차 (XYZ)
        ex, ey, ez = target[0] - curr_p.x, target[1] - curr_p.y, target[2] - curr_p.z
        
        self.int_x += ex * dt; self.int_y += ey * dt; self.int_z += ez * dt
        vx = self.kp * ex + self.ki * self.int_x + self.kd * (ex - self.pre_x) / dt
        vy = self.kp * ey + self.ki * self.int_y + self.kd * (ey - self.pre_y) / dt
        vz = self.kp * ez + self.ki * self.int_z + self.kd * (ez - self.pre_z) / dt
        self.pre_x, self.pre_y, self.pre_z = ex, ey, ez

        # 2. 회전 오차 (Yaw) - 목표 지점을 바라보도록 계산
        target_yaw = math.atan2(ey, ex)
        e_yaw = target_yaw - curr_y
        while e_yaw > math.pi: e_yaw -= 2*math.pi
        while e_yaw < -math.pi: e_yaw += 2*math.pi
        
        self.int_yaw += e_yaw * dt
        vyaw = self.kp_y * e_yaw + self.ki_y * self.int_yaw + self.kd_y * (e_yaw - self.pre_yaw) / dt
        self.pre_yaw = e_yaw
        
        return vx, vy, vz, vyaw, math.sqrt(ex**2 + ey**2 + ez**2)

    def run(self, waypoints):
        """실제 비행 루프 (메인 로직)"""
        rate = rospy.Rate(20)
        dt = 0.05
        wp_idx = 0
        
        # 연결 확인
        rospy.loginfo("FCU와 연결 대기 중...")
        while not rospy.is_shutdown() and not self.mav.current_state.connected:
            rate.sleep()
        
        # EKF 안정화 대기
        rospy.loginfo("연결 완료! 센서 안정화 대기 (5초)...")
        rospy.sleep(5.0)

        # 스트림 활성화 (Offboard 진입 필수 조건)
        for _ in range(100):
            self.mav.vel_pub.publish(TwistStamped())
            rate.sleep()

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            
            # [상태 관리] OFFBOARD 모드 전환 및 시동(ARM)
            if self.mav.current_state.mode != "OFFBOARD" and (now - last_req > rospy.Duration(5.0)):
                self.mav.set_mode_client.call(SetModeRequest(custom_mode='OFFBOARD'))
                last_req = now
            elif not self.mav.current_state.armed and (now - last_req > rospy.Duration(5.0)):
                self.mav.arming_client.call(CommandBoolRequest(value=True))
                last_req = now

            # [비행 제어]
            target = waypoints[wp_idx]
            vx, vy, vz, vyaw, dist = self.compute_pid_output(dt, target)

            # 명령 발행
            vel = TwistStamped()
            vel.header.stamp = rospy.Time.now()
            vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z = vx, vy, vz
            vel.twist.angular.z = vyaw
            self.mav.vel_pub.publish(vel)

            # [웨이포인트 전환] OFFBOARD이고 ARMED일 때만 다음 지점으로
            if self.mav.current_state.mode == "OFFBOARD" and self.mav.current_state.armed:
                if dist < 1.5:
                    rospy.loginfo(f"지점 {wp_idx} 도달: {target}")
                    
                    # 다음 지점으로 인덱스 증가, 마지막 지점이면 다시 0으로 리셋
                    wp_idx = (wp_idx + 1) % len(waypoints)
                    
                    rospy.loginfo(f"순찰 반복 중 - 다음 목표: 지점 {wp_idx} ({waypoints[wp_idx]})")
                

            rate.sleep()

# --- [메인 실행부] ---
if __name__ == "__main__":
    rospy.init_node("drone_mission_node")
    
    # 1. 웨이포인트 입력 (X, Y, Z)
    mission_path = [
        [0, 0, 5.5], [10, 0, 5.5], [10, 10, 5.5], [0, 10, 5.5], [0, 0, 5.5], [0, 5, 12.5], [0, 10, 5.5], [0, 0, 5.5]
    ]

    try:
        # 2. 미션 객체 생성 및 실행
        controller = DroneMission()
        controller.run(mission_path) # input으로 웨이포인트 전달
    except rospy.ROSInterruptException:
        pass