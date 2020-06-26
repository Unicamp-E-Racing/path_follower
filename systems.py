### SYSTEMS: contains the simulator system objects

from scipy.integrate import solve_ivp
from numpy import cos, sin, tan, arctan2, sqrt, pi
import numpy as np
import matplotlib.pyplot as plt
from constants import *


class KinematicVehicleBody(object):
    ''' 
    Vehicle Body System: half-car kinematic model.
    
    INPUTS: delta (steering angle) and v_dot (vehicle acceleration).
    STATE: body positions and velocities, yaw and yaw rate.
    OUTPUTS: body state - positions and speeds.
    '''
      
    def __init__(self, x_initial, y_initial, yaw_initial, v_initial):
        # vehicle states
        self.x = np.array([x_initial]) # x position
        self.y = np.array([y_initial]) # y position
        self.yaw = np.array([yaw_initial]) # yaw position
        self.v = np.array([v_initial + 0.00001]) # total speed
        
        # vehicle inputs
        self.delta = np.array([0]) # steering angle input
        self.v_dot = np.array([0]) # acceleration input
    
    # stores new vehicle states
    def add_states(self, new_state):
        [x, y, yaw, v] = new_state
        
        self.x = np.append(self.x, np.array([x]))
        self.y = np.append(self.y, np.array([y]))
        self.yaw = np.append(self.yaw, np.array([yaw]))
        self.v = np.append(self.v, np.array([v]))
    
    # stores new vehicle inputs
    def add_inputs(self, delta, v_dot):
        self.delta = np.append(self.delta, np.array([delta]))
        self.v_dot = np.append(self.v_dot, np.array([v_dot]))
    
    # returns the last car state
    def get_last_state(self):
        return [self.x[-1], self.y[-1], self.yaw[-1], self.v[-1]]
    
    # vehicle body model
    def model(self, t, initial_state, delta, v_dot):
        x, y, yaw, v = initial_state
        
        # saturate delta to [-30, 30] degree range
        delta = np.clip(delta, -MAX_DELTA, MAX_DELTA)
        
        # calculates the derivative of the vehicle's states
        state = [v*cos(yaw),
                 v*sin(yaw),
                 v*tan(delta)/Lb,
                 v_dot]
        
        return state

class Simulator(object):
    '''
    Simulator System: this system is responsible for simulation steps and for
    keeping track of timestamps.
    '''
    
    def __init__(self, dt):
        self.dt = dt # simulation timestep
        self.t = np.array([0]) # simulation timestamps
    
    # updates the timestamp array with a new timestamp        
    def update_time(self):
        next_time = self.t[-1] + self.dt
        self.t = np.append(self.t, np.array([next_time]))
    
    # this function outputs the value of the current simulation runtime
    def get_runtime(self):
        return self.t[-1]
    
    # this function returns an array with all the timestamps
    def get_timestamps(self):
        return self.t
    
    # vehicle body simulation step
    def simulate_step(self, Vehicle, delta, v_dot):
        
        # get initial body state
        initial_state = Vehicle.get_last_state()
        
        start_time = self.t[-1]
        end_time = start_time + self.dt
        
        # solves edo to find the next vehicle state based on inputs
        new_state = solve_ivp(Vehicle.model, (start_time, end_time),
                              initial_state, args=(delta,v_dot)).y[:,-1]
        
        # stores the inputs
        Vehicle.add_inputs(delta, v_dot)
        
        # stores and returns the calculated state
        Vehicle.add_states(new_state)
        
        # updates the simulator time
        self.update_time()
        return new_state
    