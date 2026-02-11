#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion



class PID_Controller:
	def __init__(self):
		self.current_state = State()
		self.current_pose = PoseStamped()
		self.vel_msg = TwistStamped()

		self.P_gain = 1.2
		self.I_gain = 0.02
		self.D_gain = 0.3
		self.yaw_P_gain = 1.5
		self.yaw_I_gain = 0
		self.yaw_D_gain = 0.1
		
		self.Int_x = 0
		self.err_x_pre = 0
		
		self.Int_y = 0
		self.err_y_pre = 0
		
		self.Int_z = 0
		self.err_z_pre = 0

		self.Int_yaw = 0
		self.err_yaw_pre = 0

		self.waypoints = [
            [0, 0, 5.5], 
			[10, 0, 5.5], 
			[10, 10, 5.5], 
			[0, 10, 5.5],
            [0, 0, 5.5], 
			[0, 5, 12.5], 
			[0, 10, 5.5], 
			[0, 0, 5.5]
        ]
		self.current_wp_idx = 0
		self.dist_threshold = 1

		self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
		self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
		self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
		self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


	def state_cb(self, msg):
		self.current_state = msg

	def pose_cb(self, msg):
		self.current_pose = msg
		
	def get_current_yaw(self):
		orientation_q = self.current_pose.pose.orientation
		quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
		_, _, yaw = euler_from_quaternion(quaternion)
		return yaw


	def Output(self, dt, wp):
		# x
		self.err_x= wp[0] - self.current_pose.pose.position.x
		self.Int_x = self.Int_x + (self.err_x * dt)


		V_x = self.P_gain * self.err_x + self.I_gain * self.Int_x + self.D_gain * (self.err_x - self.err_x_pre)/dt

		self.err_x_pre = self.err_x
		self.vel_msg.twist.linear.x = V_x
		
		# y
		self.err_y= wp[1] - self.current_pose.pose.position.y
		self.Int_y = self.Int_y + (self.err_y * dt)


		V_y = self.P_gain * self.err_y + self.I_gain * self.Int_y + self.D_gain * (self.err_y - self.err_y_pre)/dt

		self.err_y_pre = self.err_y
		self.vel_msg.twist.linear.y = V_y
		
		# z
		self.err_z= wp[2] - self.current_pose.pose.position.z
		self.Int_z = self.Int_z + (self.err_z * dt)


		V_z = self.P_gain * self.err_z + self.I_gain * self.Int_z + self.D_gain * (self.err_z - self.err_z_pre)/dt

		self.err_z_pre = self.err_z
		self.vel_msg.twist.linear.z = V_z

		# yaw
		target_yaw = math.atan2(self.err_y, self.err_x)
		current_yaw = self.get_current_yaw()

        # 각도 오차 계산 및 범위 정규화 (-pi ~ pi)
		self.yaw_err = target_yaw - current_yaw
		while self.yaw_err > math.pi: self.yaw_err -= 2.0 * math.pi
		while self.yaw_err < -math.pi: self.yaw_err += 2.0 * math.pi

		self.Int_yaw = self.Int_yaw + (self.yaw_err * dt)

        
		V_yaw = self.yaw_P_gain * self.yaw_err + self.yaw_I_gain * self.Int_yaw + self.yaw_D_gain * (self.yaw_err- self.err_yaw_pre)/dt

		self.err_yaw_pre = self.yaw_err
		self.vel_msg.twist.angular.z = V_yaw
		

	def get_distance(self):
		return math.sqrt(
            (self.err_x)**2 +
            (self.err_y)**2 +
            (self.err_z)**2
        )


	def run(self):
          
		rate = rospy.Rate(20)
		while not rospy.is_shutdown() and not self.current_state.connected:
			rate.sleep()

		vel_msg = TwistStamped()
		for _ in range(100):
			if rospy.is_shutdown(): break
			self.vel_pub.publish(vel_msg)
			rate.sleep()

		last_req = rospy.Time.now()
		target_wp = self.waypoints[0]

		while not rospy.is_shutdown():
			if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
				self.set_mode_client.call(SetModeRequest(custom_mode='OFFBOARD'))
				last_req = rospy.Time.now()
			elif not self.current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
				self.arming_client.call(CommandBoolRequest(value=True))
				last_req = rospy.Time.now()

			if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
				target_wp = self.waypoints[self.current_wp_idx]
				dist = self.get_distance()

				if dist < self.dist_threshold:
					self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
					rospy.loginfo(f"지점 도달! 다음 목표: {self.waypoints[self.current_wp_idx]}")


			self.Output(dt=0.05, wp=target_wp)
			self.vel_pub.publish(self.vel_msg)

			rate.sleep()
		
		
		
		
		
		
		
if __name__ == "__main__":
    rospy.init_node("velocity_yaw_node")
    try:
        controller = PID_Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass
