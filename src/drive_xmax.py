#!/usr/bin/env python
#*-* coding: utf-8 *-*

# Copyright (c) 2019 co-dirver.ai

"""
CheYou XMax toy car drive node for ROS. The vehicle steer and throttle command is very simple: 0 or 1 
"""
from enum import Enum, unique

import rospy
from drive_toy_car.drive_toy_car import _RC_Car_Driver

STOP_THROTTLE = 10.0  #如果油门开度低于这个值，玩具车将停止运动
START_THROTTLE = 12.0 #这辆玩具车从静止状态启动的时候，必须给出大于这个油门开度的值，才能够启动
MAX_STEER = 70.0    #允许的最大的方向转角,单位为度
DUR_TIME = 0.1       #启动后油门的持续时间，之后，油门将自动降为停车油门值STOP_THROTTLE

class Vehicle_status(Enum):
    """
    three staus defintion of the toy car
    """
    Stopped = 1
    Beginning_Moving = 2
    Normal_Moving = 3

class XMax_Driver(_RC_Car_Driver):
    """
    xmax toy car driver
    """
    def __init__(self):
        super(XMax_Driver,self).__init__()
        self.__XMax_status = Vehicle_status.Stopped
        self.__start_moving_time = rospy.get_time()
    
    def cmd_calibration(self,throttle,steer): 
        """
        vehicle throttle and steer calibration for carla autopilot command 
        """
        xmax_steer = steer*MAX_STEER
        if abs(throttle) < 1: 
            xmax_throttle = throttle  #如果小于1，实际上就是等于0，直接赋值,并将车辆状态设置为Stopped
            self.__XMax_status = Vehicle_status.Stopped
        else:
            if self.__XMax_status == Vehicle_status.Stopped:
                xmax_throttle = throttle*START_THROTTLE #以较高的速度启动车辆
                self.__XMax_status = Vehicle_status.Beginning_Moving
                self.__start_moving_time = rospy.get_time()
            elif self.__XMax_status == Vehicle_status.Beginning_Moving:
                t = rospy.get_time()
                delta_t = t - self._start_moveing_time
                if delta_t > DUR_TIME:
                    xmax_throttle = throttle*STOP_THROTTLE #车辆已经正常启动运行，改为按最低速度运行
                    self.__XMax_status = Vehicle_status.Normal_Moving
                else:
                    xmax_throttle = throttle*START_THROTTLE #继续按启动速度运行,状态保持不变
            elif self.__XMax_status == Vehicle_status.Normal_Moving:
                xmax_throttle = throttle*STOP_THROTTLE #继续按最低速度运行，状态保持不变       
        return xmax_throttle,xmax_steer

if __name__ == '__main__':
    toy_car_driver = XMax_Driver()
    toy_car_driver.spin()
