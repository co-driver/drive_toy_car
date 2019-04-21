#!/usr/bin/env python
import rospy
import Adafruit_PCA9685 
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from cheyou_toy_car.msg import MsgDriverCmd

pwm = Adafruit_PCA9685.PCA9685() 
pwm.set_pwm_freq(100) 
steer_channel=12
throttle_channel=15


def drive_with_cmd(throttle,steer):
    throttle_pulse_width=fix(2.048*throttle+614.6)
    steer_pulse_width=fix(2.048*steer+614.6)    
    rospy.loginfo(rospy.get_caller_id() + " throttle_pulse_width=%d steer_pulse_width=%d",\
        throttle_pulse_width,steer_pulse_width)                                              
    pwm.set_pwm(throttle_channel,0, throttle_pulse_width) 
    pwm.set_pwm(steer_channel,0, steer_pulse_width)



def callback(VehicleStatus):
    throttle=VehicleStatus.control.throttle
    steer=VehicleStatus.control.steer
    rospy.loginfo(rospy.get_caller_id() + " throttle=%d steer=%d", throttle,steer)
    drive_with_cmd(throttle,steer)


def drive_it():
        rospy.init_node("raspberry_driver")
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status",CarlaEgoVehicleStatus,callback)
        rospy.spin()

if __name__ == '__main__':
    drive_it()        
