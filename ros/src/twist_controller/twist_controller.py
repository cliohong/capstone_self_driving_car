from lowpass import LowPassFilter
from pid import PID
import numpy as np


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        
        self.wheel_radius = args[1]
        self.accel_limit = args[2]
        self.decel_limit = args[3]
        self.brake_deadband = args[4]
        self.fuel_capacity = args[5]
        self.max_throttle_percentage = args[6]
        self.max_braking_percentage = args[7]
        self.steer_ratio=args[8]
        self.max_acc = args[9]
        self.vehicle_mass = args[0] + self.fuel_capacity*GAS_DENSITY
        self.init_time = 0.02
        self.lowpss = LowPassFilter(self.accel_limit,self.init_time)
        self.pid = PID(2., 0.5, 0.15, mn=self.max_braking_percentage, mx=self.max_throttle_percentage)
        #max torque corresponding to max throttle value of 1.0
        self.max_acc_torque = self.vehicle_mass* self.max_acc *self.wheel_radius
        #max brake torque corresponding to deceleration limit
        self.max_brake_torque = self.vehicle_mass * abs(self.decel_limit)*self.wheel_radius

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        limit_time = 0.2
        self.dbw_enabled = args[3]
        if self.dbw_enabled:
            self.target_linear_velocity = args[0]
            self.target_angular_velocity = args[1]
            self.curr_linear_velocity = args[2]
            self.cte = args[4]
            steer =self.pid.step(self.cte, self.init_time)
            throttle = 0.0
            brake = 0.0
            #calculate vel difference that needs to modify
            error = self.target_linear_velocity - self.curr_linear_velocity
            #apply limits to acceleration
            acceleration = error / limit_time
            
            if acceleration > 0 :
                acceleration = min(self.accel_limit, acceleration)
            else:
                acceleration = max(self.decel_limit,acceleration)
        
        #under deadband condition, there is no controls
            if abs(acceleration)<self.brake_deadband:
                return throttle, brake, steer

            #calculate torque = M * acc *R
            torque = self.vehicle_mass * acceleration * self.wheel_radius

            if torque >0 :
                throttle, brake = min(1.,torque/self.max_acc_torque), 0.0
            else:
                throttle, brake = 0.0, min(abs(torque),self.max_brake_torque)

            return throttle,brake, steer
        
        else:
            self.pid.reset()

        return 0., 0., 0.

