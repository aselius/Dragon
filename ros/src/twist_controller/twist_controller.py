import rospy # For time

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, **kwargs):
        self.last_time = None

	self.vehicle_mass = kwargs['vehicle_mass']
	self.fuel_capacity = kwargs['fuel_capacity']
	self.brake_deadband = kwargs['brake_deadband']
	self.decel_limit = kwargs['decel_limit']
	self.accel_limit = kwargs['accel_limit']
	self.wheel_radius = kwargs['wheel_radius']
	self.wheel_base = kwargs['wheel_base']
	self.steer_ratio = kwargs['steer_ratio']
	self.max_lat_accel = kwargs['max_lat_accel']
	self.max_steer_angle = kwargs['max_steer_angle']
	self.min_speed = kwargs['min_speed']

	self.yaw_controller = YawController(kwargs['wheel_radius'], kwargs['steer_ratio'], kwargs['min_speed'], kwargs['max_lat_accel'], kwargs['max_steer_angle'])

	self.pid = PID(1, 0.01, 0.01, self.decel_limit, self.accel_limit);
	self.sample_time = 1.0/50; # 50 Hz
        self.tau_correction = 0.2
        self.ts_correction = 0.1
	self.low_pass_filter = LowPassFilter(self.tau_correction, self.ts_correction)

    def clear(self):
        self.last_time = None

    def control(self, desired_vel, current_vel):

        error = current_vel - desired_vel
        '''
        p_term = self.pid.kp * error
        d_term = 0
        cur_time = rospy.get_time()
        if self.last_time is not None:
            diff_time = cur_time - self.last_time
            rospy.logerr("Cur_time: %f, Diff_time: %f", cur_time, diff_time)
            d_term = self.pid.kd * (error - self.last_err)/(diff_time)
            rospy.logerr("D term: %f", d_term)
        throttle = p_term + d_term
        '''
	throttle = self.pid.step(error, self.sample_time)
        brake = 0.0
        '''
        if throttle > 1.0:
            throttle = 1.0
        if throttle < -1.0:
            throttle = -1.0
        # If its negative throttle, then reverse
        if throttle < 0:
            brake = 5 * throttle
            throttle = 0.0
        '''
	if throttle < 0:
	    brake = -2000*throttle + 1000
	    throttle = 0.0
        #self.last_err = error
        #self.last_time = cur_time

        return throttle, brake
