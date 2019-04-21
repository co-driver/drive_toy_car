#!/usr/bin/env python
import rospy
from cheyou_toy_car.msg import MsgDriverCmd

def send_cmd():
    var_DriverCmd=MsgDriverCmd()
    pub = rospy.Publisher('car_cmd', MsgDriverCmd, queue_size=1)
    rospy.init_node('tel_op')
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        var_DriverCmd.throttle_pos=10
        var_DriverCmd.steer_pos=0
        pub.publish(var_DriverCmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_cmd()
    except rospy.ROSInterruptException:
        pass
