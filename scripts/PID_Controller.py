#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest




class PID_Controller:
	def __init__(self, P_gain, I_gain, D_gain, x, y, z):
		self.current_state = State()
		self.current_pose = PoseStamped()
		self.vel_msg = TwistStamped()

		self.P_gain = P_gain
		self.I_gain = I_gain
		self.D_gain = D_gain
		
		self.Int_x = 0
		self.err_x_pre = 0
		
		self.Int_y = 0
		self.err_y_pre = 0
		
		self.Int_z = 0
		self.err_z_pre = 0

		self.target_wp = [x, y, z]

		self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
		self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
		self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
		self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
		self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


	def state_cb(self, msg):
		self.current_state = msg

	def pose_cb(self, msg):
		self.current_pose = msg
		
		
		
	def Output(self, dt):
		# x
		err_x= self.target_wp[0] - self.current_pose.pose.position.x
		self.Int_x = self.Int_x + (err_x * dt)


		V_x = self.P_gain * err_x + self.I_gain * self.Int_x + self.D_gain * (err_x - self.err_x_pre)/dt

		self.err_x_pre = err_x
		self.vel_msg.twist.linear.x = V_x
		
		# y
		err_y= self.target_wp[1] - self.current_pose.pose.position.y
		self.Int_y = self.Int_y + (err_y * dt)


		V_y = self.P_gain * err_y + self.I_gain * self.Int_y + self.D_gain * (err_y - self.err_y_pre)/dt

		self.err_y_pre = err_y
		self.vel_msg.twist.linear.y = V_y
		
		# z
		err_z= self.target_wp[2] - self.current_pose.pose.position.z
		self.Int_z = self.Int_z + (err_z * dt)


		V_z = self.P_gain * err_z + self.I_gain * self.Int_z + self.D_gain * (err_z - self.err_z_pre)/dt

		self.err_z_pre = err_z
		self.vel_msg.twist.linear.z = V_z
		
		
	def run(self):
          
		# Setpoint publishing MUST be faster than 2Hz
		rate = rospy.Rate(20)

    	# Wait for Flight Controller connection
		while(not rospy.is_shutdown() and not self.current_state.connected):
			rate.sleep()

		# Send a few setpoints before starting
		for i in range(100):
			if(rospy.is_shutdown()):
				break

			self.vel_pub.publish(self.vel_msg)
			rate.sleep()

		offb_set_mode = SetModeRequest()
		offb_set_mode.custom_mode = 'OFFBOARD'

		arm_cmd = CommandBoolRequest()
		arm_cmd.value = True

		last_req = rospy.Time.now()

		while(not rospy.is_shutdown()):
			if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
				if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
					rospy.loginfo("OFFBOARD enabled")

				last_req = rospy.Time.now()
			else:
				if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
					if(self.arming_client.call(arm_cmd).success == True):
						rospy.loginfo("Vehicle armed")

					last_req = rospy.Time.now()


			self.Output(dt=0.05)
			self.vel_pub.publish(self.vel_msg)

			rate.sleep()
		
		
		
		
		
		
		
if __name__ == "__main__":
    rospy.init_node("velocity_yaw_node")
    try:
        controller = PID_Controller(1.0, 0.1, 0.01, 0, 0, 2)
        controller.run()
    except rospy.ROSInterruptException:
        pass
