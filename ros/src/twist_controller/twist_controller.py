from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # Implement
        self.yawC = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.steerPID = PID(.4,.001,.05,mn=-.5*max_steer_angle, mx=.5*max_steer_angle)
        self.targetAngLPF = LowPassFilter(3.,1.)
        self.throttlePID = PID(.6, .01, .3, mn=0, mx=1.)
        self.breakPID = PID(.5, .01, .1, mn=0, mx=1000.)

    def control(self, target_lin_vel, target_ang_vel, current_lin_vel):
        steer = self.yawC.get_steering(target_lin_vel, self.targetAngLPF.filt(target_ang_vel), current_lin_vel)        
        steer_pid = self.steerPID.step(steer,1)

        if target_lin_vel == 0:
            break_pid = 1000.
            throttle_pid = 0
            self.throttlePID.reset()
        elif (current_lin_vel-target_lin_vel) > -1:
            break_pid = self.breakPID.step( 125*(current_lin_vel-target_lin_vel+1), .05)
            #throttle_pid = max(0.1,1 - abs(steer_pid) - self.throttlePID.step( (current_lin_vel-target_lin_vel+1), .05))
            throttle_pid = .1
        else:
            break_pid = 0
            # When starting from rest, have a lower throttle speed
            if current_lin_vel < 3.:
                throttle_pid = self.throttlePID.step(.4, 1)
            else:
                throttle_pid = self.throttlePID.step( (1 - abs(steer_pid/self.steerPID.max)), 1)
        return throttle_pid, break_pid, steer_pid 


class SiteController(Controller):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        Controller.__init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.steerPID = PID(.8,.0,.8,mn=-.9*max_steer_angle, mx=.9*max_steer_angle)

    def control(self, target_lin_vel, target_ang_vel, current_lin_vel):
        steer = self.yawC.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)        
        steer_pid = self.steerPID.step(steer*2,1)

        if target_lin_vel == 0:
            break_pid = 1000.
            throttle_pid = 0
            self.throttlePID.reset()
        elif (current_lin_vel-target_lin_vel) > -1:
            break_pid = self.breakPID.step( 125*(current_lin_vel-target_lin_vel+1), .05)
            # throttle_pid = max(0.1,1 - abs(steer_pid) - self.throttlePID.step( (current_lin_vel-target_lin_vel+1), .05))
            throttle_pid = .1
        else:
            break_pid = 0.
            #break_pid = self.breakPID.step( 50*abs(steer_pid/self.steerPID.max), .05)
            throttle_pid = .15 + .05 * (1 - abs(steer_pid/self.steerPID.max))
        return throttle_pid, break_pid, steer_pid 
