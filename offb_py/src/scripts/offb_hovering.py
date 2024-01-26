#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State

def state_cb(msg):
	global current_state
	current_state = msg

if __name__=="__main__":
	current_state = State()
	rospy.init_node('offb_node', anonymous=True) # 노드 초기화

	""" [ rospy.Subscriber ] 
	rospy.Subscriber(Name,data_class,callback, callback_args, queue_size, buff_size, tcp_nodelay)
	- name은 구독할 토픽명, data_class는 해당 토픽의 데이터 타입
	- callback 변수는 토픽이 발행되는 이벤트가 발생했을때 작동할 이벤트 리스너 함수를 콜백 함수의 형태로 요구
	기본 argument는 구독한 메시지 객체 
	"""
	rospy.Subscriber("mavros/state", State, state_cb)
	
	""" [ rospy.Publisher ]
	rospy.Publisher(Name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)
	- 필수는 Name, data_class, queue_size
	- Name은 Publisher가 발행하는 토픽의 이름
	- data_class는 발행할 메시지의 타입(클래스)
	- queue_size는 데이터의 큐 사이즈
	- 큐 사이즈는 발행되는 메시지를 얼마나 가지고 있을지에 관련된 변수, 신규 데이터가 들어오는 경우
	오래된 데이터부터 삭제하게 된다. 적합한 값을 선정하는것은 개발자의 몫
	"""

	local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
	local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

	rospy.loginfo("Publisher and Subscriber Created")
	arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
	rospy.loginfo("Clients Created")

	# Setpoint publishing MUST be faster than 2 hz
	rate = rospy.Rate(20)
	
	# Wait for FC connection
	while(not rospy.is_shutdown() and not current_state.connected):
		print(current_state.connected)
		rate.sleep()
	rospy.loginfo("Creating pose")
	pose = PoseStamped()

	#set position here
	pose.pose.position.x = 0
	pose.pose.position.y = 0
	pose.pose.position.z = 2

	# set velocity here
	"""
	vel = Twist()
	vel.linear.x = 20
	vel.linear.y = 20
	vel.linear.z = 20
	"""
	
	pos_loginfo_msg = "position = " + str(pose.pose.position.x) + " " + str(pose.pose.position.y)+" "+ str(pose.pose.position.z)
	rospy.loginfo(pos_loginfo_msg)

	# Send a few setpoints before starting
	for i in range(100):
		if(rospy.is_shutdown()):
			break
		local_pos_pub.publish(pose)
		rate.sleep()
	
	rospy.loginfo("Creating Objects for services")
	offb_set_mode = SetModeRequest()
	offb_set_mode.custom_mode = "OFFBOARD"
	arm_cmd = CommandBoolRequest()
	arm_cmd.value = True
	
	last_request = rospy.Time.now()
	
	while(not rospy.is_shutdown()):
		#print(current_state)
		if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
			if(set_mode_client.call(offb_set_mode).mode_sent == True):
				rospy.loginfo("OFFBOARD Enabled")

			last_request = rospy.Time.now()
	
		elif (not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
			# 위와 다른방법으로 Service call 을 할수 있음. 결과는 동일
			arm_client_1 = arming_client(arm_cmd.value)
			if arm_client_1.success:
				rospy.loginfo("Vehicle ARMED")
			last_request = rospy.Time.now()
			
		local_pos_pub.publish(pose)
		#local_vel_pub.publish(vel)
		#print current_state