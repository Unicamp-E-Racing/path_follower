### CONTROLLERS: contains the simulator controller functions

from numpy import sin, cos, tan, arctan, pi
import numpy as np
from constants import *

def steeringController(x, y, yaw, v, x_target, y_target, yaw_target):
    ''' 
    E-Racing's Steering Controller
    
    INPUTS: vehicle states, position and steering targets.
    OUTPUTS: steering angle.
    '''
    # define controller hyperparameters
    l = 1
    g = v
    k = v
    b = 2*Lb/tan(MAX_DELTA)
    
    # calculate sigma
    s = (y - y_target)*cos(yaw_target) - (x - x_target)*sin(yaw_target) + \
        b*wrap2pi(yaw-yaw_target)         
    
    # calculate the steering angle
    yaw_dot = -(Lb/b)*sin(yaw-yaw_target) - ((k*Lb)/(b*v))*(s/(l + abs(s)))
    delta = arctan(yaw_dot)
    
    return delta

def accelerationController(x, y, yaw, v, x_target, y_target, yaw_target, v_target):
    '''
    E-Racing's Acceleration Controller
    
    INPUTS: vehicle states, position and speed targets.
    OUTPUTS: steering angle.
    '''
    # define controller hyperparameters
    l = 1
    g = v
    k = v
    z = 2*v/MAX_ACCELERATION
    
    # calculate sigma
    s = (y - y_target)*cos(yaw_target) - (x - x_target)*sin(yaw_target) + \
        z*(v - v_target)
    
    # calculate the acceleration
    v_dot = -(v/z)*sin(yaw - yaw_target) - ((k)/(z))*(s/(l + abs(s)))
    
    return v_dot

# this helper function wraps an angle to the interval [-pi,pi]
def wrap2pi(angle):
    wrapped_angle = np.remainder(angle, 2*pi)
    if wrapped_angle > pi:
        wrapped2pi = -2*pi + wrapped_angle
    else:
        wrapped2pi = wrapped_angle
    return wrapped2pi

    