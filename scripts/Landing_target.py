#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def target_mover():
    rospy.init_node('target_mover', anonymous=True)
    pub = rospy.Publisher('/target/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10Hz

    msg = Twist()
    msg.linear.x = 0.4  # 초속 0.5m로 전진
    msg.angular.z = 0.0 # 약간씩 회전 (원운동)

    rospy.loginfo("상자 자동 주행 시작!")
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try: target_mover()
    except rospy.ROSInterruptException: pass
