from lowpass import LowPassFilter
from pid import PID
import numpy as np


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.accel_limit = args[0]
        self.max_throttle_percentage = args[1]
        self.max_braking_percentage = args[2]
        self.steer_ratio=args[3]
        self.init_time = 0.03
        self.lowpss = LowPassFilter(self.accel_limit,self.init_time)
        #override filter. it includes lag
        self.lowpss= None
        self.pid = PID(2.0, 0.4, 0.1, mn=self.max_braking_percentage, mx=self.max_throttle_percentage)
        
        

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        self.dbw_enabled = args[3]
        if self.dbw_enabled:
            self.target_linear_velocity = args[0]
            self.target_angular_velocity = args[1]
            self.curr_linear_velocity = args[2]
            self.cte = args[4]
            steer =self.pid.step(self.cte, self.init_time)
#            steer = self.target_angular_velocity*self.steer_ratio
            throttle = 0.0
            brake = 0.0
            if abs(self.target_linear_velocity > abs(self.curr_linear_velocity)):
                if self.target_linear_velocity < 0:
                    throttle = -0.01
                else:
                    throttle_thresh = np.min(4*[1 - ((self.curr_linear_velocity - 0.1)/self.target_linear_velocity)],self.max_throttle_percentage)
                    throttle = np.max(throttle_thresh,self.max_braking_percentage)

            elif self.curr_linear_velocity > 0.1:

                throttle_thresh = np.min(4*[1-((self.curr_linear_velocity+0.1)/self.target_linear_velocity)],self.max_throttle_percentage)
                throttle = np.max(throttle_thresh, self.max_braking_percentage)
            else:
                throttle = - 0.01
                    
            if throttle < 0. :
                brake = -throttle
                throttle = 0.
            return throttle,brake, steer
        
        else:
            self.pid.reset()
                    
        return 0., 0., 0.
