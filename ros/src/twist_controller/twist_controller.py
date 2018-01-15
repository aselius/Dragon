import rospy # For time

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704 # NOTE(aselius): This is conversion from milesph to m/s


class Controller(object):
    def __init__(self, **kwargs):
        self.last_time = None

	self.vehicle_mass    = kwargs['vehicle_mass']
	self.fuel_capacity   = kwargs['fuel_capacity']
	self.brake_deadband  = kwargs['brake_deadband']
	self.decel_limit     = kwargs['decel_limit']
	self.accel_limit     = kwargs['accel_limit']
	self.wheel_radius    = kwargs['wheel_radius']
	self.wheel_base      = kwargs['wheel_base']
	self.steer_ratio     = kwargs['steer_ratio']
	self.max_lat_accel   = kwargs['max_lat_accel']
	self.max_steer_angle = kwargs['max_steer_angle']
	self.min_speed       = kwargs['min_speed']

	self.total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

	self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
					    kwargs['min_speed'], kwargs['max_lat_accel'],
					    kwargs['max_steer_angle'])

	self.pid = PID(3, 0, 0.001, self.decel_limit, self.accel_limit);
        self.tau_correction = 0.2
        self.ts_correction = 0.1
	
	self.low_pass_filter = LowPassFilter(self.tau_correction, self.ts_correction)

	# NOTE(aselius): I believe this is just 50hz, but should be fine for now..
	# self.sample_time = 1.0/50; # 50 Hz
	self.last_time = rospy.get_time()

    def reset(self):
        self.velocity_pid.reset()

    def control(self, proposed_linear_velocity, proposed_angular_velocity,
		current_linear_velocity, dbw_enabled):
        # DONE: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        curr_time = rospy.get_time()
        sample_time = curr_time - self.last_time
        self.last_time = curr_time
        error = proposed_linear_velocity - current_linear_velocity
        accel = self.pid.step(error, sample_time)
        accel = self.lowpass_pass_filter.filt(accel)

        steer = self.yaw_controller.get_steering(proposed_linear_velocity,
						 proposed_angular_velocity,
						 current_linear_velocity)
	# NOTE(aselius): I believe we don't need a lowpass for steering
	# since the lowpass is trying to adjust for measurement noise
	# coming from the car, which is only limited to current_velocity.

        if accel > 0.:
            throttle = accel
            brake = 0.
        else:
            throttle = 0.
            # brake in units of torque (N*m). 
            # The correct values for brake can be computed 
            # using the desired acceleration, weight of the vehicle, and wheel radius.
            # if proposed_linear_velocity is too small, make full brake
            if proposed_linear_velocity < 0.1:
                brake = abs(self.decel_limit) * self.total_mass * self.wheel_radius
            else:
                decel = abs(accel)
                if decel < self.brake_deadband:
                    decel = 0.
                brake = decel * self.total_mass * self.wheel_radius

        return throttle, brake, steer

