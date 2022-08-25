import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 loop_rate, 
                 vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.throttle_PID = PID(
            kp = 0.3, # Proportional gain
            ki = 0.1, # Integral gain
            kd = 0., # Derivative gain
            mn = 0., # Min throttle saturation
            mx = 0.2 # Max throttle saturation
        )
        
        # Filter noisy raw velocity signal
        self.velocity_LPF = LowPassFilter(
            tau = 0.5, # Cutoff frequency = 1/(2pi*tau)
            ts = 0.02 # 1.0/loop_rate  # Sample time
        )
        
        self.yaw_controller = YawController(
            wheel_base = wheel_base,
            steer_ratio = steer_ratio,
            min_speed = 0.1,
            max_lat_accel = max_lat_accel,
            max_steer_angle = max_steer_angle)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

    def control(self, sample_time, prop_vel_lin, prop_vel_ang, curr_vel_lin):
        # TODO: Change the arg, kwarg list to suit your needs
        filt_curr_vel_lin = self.velocity_LPF.filt(curr_vel_lin)
        
        steering = self.yaw_controller.get_steering(prop_vel_lin, prop_vel_ang, curr_vel_lin) # filt_curr_vel_lin
        # TODO: Consider dampening steering as suggested in tutorial
        
        vel_error = prop_vel_lin - filt_curr_vel_lin
        self.last_vel_lin = filt_curr_vel_lin
        
        throttle = self.throttle_PID.step(vel_error, sample_time)
        brake = 0
        
        if prop_vel_lin == 0 and filt_curr_vel_lin < 0.1:
            throttle = 0
            brake = 700 # Nm - To hold car still when stopped. Acceleration ~ 1m/s^2
            
        elif throttle < 0.1 and vel_error < 0.1:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Conversion to Nm
        
        # Return throttle, brake, steer
        return throttle, brake, steering
    
    def reset(self):
        rospy.logwarn("Reset twist_controller")
        # TODO: Reset PID and last execution time before re-activation
        self.throttle_PID.reset()
        
