#!/usr/bin/env python
import rospy
import Adafruit_PCA9685
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus


class _RC_Car_Driver(object):
    """ RC car controller using PCA9685

    An object of class _RC_Car_Driver is a node that controls the steer serv and ESC
    of a rc toy car.
    """    
    
    _DEF_CMD_TIMEOUT = 0.1
    _STEER_CH = 12
    _THROT_CH = 15
    _PWM_FREQ = 100

    def __init__(self):           
	
	rospy.init_node("rc_car_driver")
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status",CarlaEgoVehicleStatus,self.callback)

        # Command timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout",
                                                      self._DEF_CMD_TIMEOUT))
        except:
            rospy.logwarn("The specified command timeout value is invalid. "
                          "The default timeout value will be used instead.")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT

        # pwm frequency: should be 50 or 100
        try:
            self._pwm_frequency = float(rospy.get_param("~pwm_requency",
                                             self._PWM_FREQ))
            if self._pwm_frequency < 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified pwm frequency is invalid. "
                          "The default pwm frequency will be used instead.")
            self._throttle_channel = self._PWM_FREQ

        # steer channel
        try:
            self._steer_channel = float(rospy.get_param("~steer_channel",
                                             self._STEER_CH))
            if not isinstance(self._steer_channel,int):
                raise ValueError()
        except:
            rospy.logwarn("The specified steer channel is invalid. "
                          "The default steer channel will be used instead.")
            self._steer_channel = self._STEER_CH

        # throttle channel
        try:
            self._throttle_channel = float(rospy.get_param("~throttle_channel",
                                             self._STEER_CH))
            if not isinstance(self._throttle_channel,int):
                raise ValueError()
        except:
            rospy.logwarn("The specified throttle channel is invalid. "
                          "The default throttle channel will be used instead.")
            self._throttle_channel = self._THROT_CH




        self._pwm = Adafruit_PCA9685.PCA9685() 
        self._pwm.set_pwm_freq(self._pwm_frequency) 

        # _last_cmd_time is the time at which the most recent 
        # driving command was received.        
        self._last_cmd_time = rospy.get_time()

	self._throttle = 0.0
	self._steer = 0.0


    def callback(self,VehicleStatus):
        t = rospy.get_time()
        delta_t = t - self._last_cmd_time
        self._last_cmd_time = t

        if (self._cmd_timeout > 0.0 and
            t - self._last_cmd_time > self._cmd_timeout):
            # Too much time has elapsed since the last command. Stop the
            # vehicle.
            self._throttle = 0.0
            self._steer = 0.0
        elif delta_t > 0.0:
            self._throttle=VehicleStatus.control.throttle
            self._steer=VehicleStatus.control.steer
	else:
	    pass
        


	rospy.loginfo(rospy.get_caller_id() + " self._throttle=%f self._steer=%f", self._throttle,self._steer)
	throttle,steer = self.cmd_calibration(self._throttle,self._steer)
        rospy.loginfo(rospy.get_caller_id() + " throttle=%f steer=%f", throttle,steer)

        throttle_pulse_width=int(204.8*throttle+614.6)
        steer_pulse_width=int(204.8*steer+614.6)    
        rospy.loginfo(rospy.get_caller_id() + " throttle_pulse_width=%d steer_pulse_width=%d",\
            throttle_pulse_width,steer_pulse_width)                                              
        self._pwm.set_pwm(self._throttle_channel,0, throttle_pulse_width) 
        self._pwm.set_pwm(self._steer_channel,0, steer_pulse_width)        

    def cmd_calibration(self,throttle,steer):
	return throttle,steer

    def spin(self):
	rospy.spin()

if __name__ == '__main__':
    driver = _RC_Car_Driver()
    driver.spin()
