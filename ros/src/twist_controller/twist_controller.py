import rospy # For time

from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, Kp, Ki, Kd):
        #self.last_err = 0.0
        #self.acc_err = 0.0
        #self.last_time = None
	self.pid = PID(Kp, Ki, Kd);
	self.sample_time = 1.0/50;

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
