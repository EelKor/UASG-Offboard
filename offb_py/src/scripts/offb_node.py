#! /usr/bin/env python3

import rospy
import time
import datetime
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# rrt_star 파일의 변수 3개 끌고옴
from rrt_star import path_planning_node_x,path_planning_node_y,num_node

# 현재 내 드론 상태랑 Position을 알아오기 위함. position Message type은 PoseStamped에 있음
current_state = State()
current_position = PoseStamped()

# Terminal창에 한번만 info를 띄우기 위해 만든 트리거, whule문 안에서 loginfo가 계속 껐다 켜지는데 trigger조건으로 한번만 켜지게 함 
log_info_trigger = False

# Callback 함수. Subscribe할 때 Callback 함수가 필요함. 
# 예시: Pose_callback 함수 내 정의도어 있는 전역변수 current_position을 호출하면 현재 내 위치를 알 수 있음  
def state_cb(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
	global current_position
	current_position = msg

# 내가 가야할 waypoint랑 현재 내 위치의 차이를 계산해서 threshold 값 내로 x,y,z축이 들어오면 True를 반환하는 함수임
# Waypoint에 도착했다는 것을 알기 위해 만든 함수. 여기서 Callback 함수가 어떻게 쓰이는지 확인할 수 있음 
# current_pos 라는 매개변수에 current_position 을 입력하면  PoseStamped.pose.position.x , y , z를 알수 있음
def is_converged(target_pose, current_pos, threshold=0.05):
    return(
        abs(target_pose.pose.position.x - current_pos.pose.position.x) < threshold and
        abs(target_pose.pose.position.y - current_pos.pose.position.y) < threshold and
        abs(target_pose.pose.position.z - current_pos.pose.position.z) < threshold 
    )

# 파이썬 파일이 직접 실행됬을 때만 작동하는 main 문. 다른 파일에서 import하면 해당 main 문은 작동 X
if __name__ == "__main__":
    # Node 초기화
    rospy.init_node("offb_node_py", anonymous=True)

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    # Subscriber을 정의함. 해당 Node 이름이 pose_sub이고 topic은 "mavros/local_position/pose"를 받고 있음, 
    # PoseStamped 메세지 타입을 받을 것이며 Callback함수로는 위에서 정의한 pose_callback을 실행할 것임
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pose_callback)
    
    # Publisher Node 생성임. 토픽은 "mavros/setpoint_position/local"을 발행, queue_size=10은 버퍼임 
    # Offbaord 모드 Waypoint 항법에서 가장 중요한 퍼블리셔 노드임. 여기에 값을 입력하면 거기로 추종해서 드론이 움직임 
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
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

    is_mynode_num = 0
    event = 0
    # Ros.spin 대신 while문으로 main 동작 실행 
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            #offboard mode랑 arming 실행시켜줌 set_mode_client.call(offb_set_mode.mode_sent == True)가 offboard 모드 실행
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        # takeoff 완료하면 실행됨. event = 1이 되며 waypoint 항법 시작. 그리고 시간 재기 시작함
        if is_converged(pose, current_position):  
            if is_mynode_num == 1:
                start = time.time()
            event = 1
        # take off 안했을 때, local_pos_pub.publish(pose)로 pose 데이터를 publish해줌  
        if event == 0:
            local_pos_pub.publish(pose)
            # loginfo 한번 하고 trigger로 인해 한번만 info 됨)
            if not log_info_trigger:
                rospy.loginfo("Take_off")
                log_info_trigger = True

        # Takeoff 하면 waypoint항법 시작
        elif event == 1:
            if log_info_trigger:
                rospy.loginfo("Waypoint Guidance and Node Number = 1")
                log_info_trigger = False
            
            # rrt_star.py 파일에 입력된 경로 list를 순서대로 Indexing하면서 Publih할 pose에 할당
            pose.pose.position.x = float(path_planning_node_x[is_mynode_num])
            pose.pose.position.y = float(path_planning_node_y[is_mynode_num])
            pose.pose.position.z = 2

            local_pos_pub.publish(pose)  #  Publish 실행

            # 현재 경로 리스트 인덱스 넘버가 경로 list 총 갯수보다 작으면서 현재 위치가 추종 위치 근처에 있을때 실행,
            # 즉 waypoint에 도착했을때 실행됨 
            if (is_converged(pose, current_position) and is_mynode_num < num_node):
                is_mynode_num = is_mynode_num + 1
                rospy.loginfo("Node Number = %d", is_mynode_num)
                rospy.loginfo("X = %3f, Y= %3f",pose.pose.position.x, pose.pose.position.y)

            # 마지막 waypoint에 도착했을 때, Landing 모드로 변경
            if (is_converged(pose, current_position) and is_mynode_num == num_node):
                offb_set_mode.custom_mode = 'AUTO.LAND' 
                set_mode_client.call(offb_set_mode).mode_sent == True
                rospy.loginfo("LAND Mode")
                end = time.time()
                sec = (end - start)
                # 시간 측정함
                if not log_info_trigger:
                    rospy.loginfo(str(datetime.timedelta(seconds=sec)))
                    log_info_trigger = True
                    # Landing 끝나고 arming 끝났을때 가제보 종료
                    while True:
                        if not current_state.armed:
                            rospy.signal_shutdown
                        rate.sleep()
        rate.sleep()
        