#! /usr/bin/env python

import rospy
import math # 수학 계산용 (필수!)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

# ★ [핵심] 각도(도)를 넣으면 쿼터니언(z, w)을 뱉어주는 마법의 함수
def get_yaw_orientation(degree):
    # 1. 도(Degree)를 라디안(Radian)으로 변환
    radian = math.radians(degree)
    
    # 2. 쿼터니언 공식 (Yaw 회전만 할 때)
    # x=0, y=0, z=sin(각도/2), w=cos(각도/2)
    q_z = math.sin(radian / 2.0)
    q_w = math.cos(radian / 2.0)
    
    return q_z, q_w

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    
    # 초기 방향 설정 (0도 = 동쪽)
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1 # w는 기본값이 1이어야 함 (중요!)

    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    start_time = 0

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        if current_state.armed:
            if start_time == 0:
                start_time = rospy.Time.now().to_sec()
            
            elapsed = rospy.Time.now().to_sec() - start_time


            if elapsed < 5.0:

                pose.pose.position.x = 0
                pose.pose.position.y = 0
                

                qz, qw = get_yaw_orientation(0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

            elif elapsed < 10.0:

                pose.pose.position.x = 5
                pose.pose.position.y = 0
                
                qz, qw = get_yaw_orientation(0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

            elif elapsed < 15.0:
                pose.pose.position.x = 5
                pose.pose.position.y = 5
                
                qz, qw = get_yaw_orientation(90)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

            elif elapsed < 20.0:
                pose.pose.position.x = 0
                pose.pose.position.y = 5
                

                qz, qw = get_yaw_orientation(180)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

            else:

                pose.pose.position.x = 0
                pose.pose.position.y = 0
                

                qz, qw = get_yaw_orientation(270)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

            pose.pose.position.z = 2

        local_pos_pub.publish(pose)
        rate.sleep()
