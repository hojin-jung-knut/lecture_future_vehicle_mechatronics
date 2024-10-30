#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd

class S_Drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        #/ctrl_cmd 토픽 발행
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        rate = rospy.Rate(30) # 30Hz
        #CtrlCmd 메세지 생성
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = 0.1
        # cmd.velocity = 5
        steering_cmd = (-0.1,0.1) # wheel steer angle(rad)
        # steering gear ratio = 13.914
        cmd_cnts = 50 #토픽 전송 수
        
        while not rospy.is_shutdown():
            for i in range(2):
                # 0 : 우회전, 1 : 좌회전
                print("s_drive cmd")
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    #발행
                    cmd_pub.publish(cmd)
                    rate.sleep()

if __name__ == '__main__':
    try:
        s_d=S_Drive()
    except rospy.ROSInterruptException:
        pass