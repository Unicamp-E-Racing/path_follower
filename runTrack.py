### RUN TRACK: simulates path following of a complete race track

import systems as sys
import controllers as ctrl
import plots
import numpy as np
from trackGenerator import genSimpleTrack
from constants import *

# function checks if vehicle is close to the target
def is_close(x, y, x_target, y_target):
    # calculate the distance between the vehicle and the taret
    dist = np.sqrt((x-x_target)**2 + (y-y_target)**2)
    
    # check if dist is smaller than minimum distance
    if dist < MIN_DIST:
        return True
    else:
        return False
  
# function to calculate the radius of the trajectory
def calculate_radius(previous_point, current_point, next_point):
    x1, y1 = previous_point
    x2, y2 = current_point
    x3, y3 = next_point
    
    a = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    b = np.sqrt((x3-x2)**2 + (y3-y2)**2)
    c = np.sqrt((x3-x1)**2 + (y3-y1)**2)
    
    q = (a**2 + b**2 - c**2)/(2*a*b)

    # checks q value to prevent division by 0
    if abs(q) == 1:
        r = MAX_RADIUS
    else:
        r = c/(2*np.sqrt(1-q**2))
    
    return r
    
# generates a random race track
blue_cones, yellow_cones = genSimpleTrack()

# calculate the middle track points, which we will follow
middle_points = (yellow_cones + blue_cones)/2

# finds the initial point
x_initial, y_initial = middle_points[0]
x_next, y_next = middle_points[1]

# calculates the initial yaw
yaw_initial = np.arctan2((y_next - y_initial),
                         (x_next - x_initial))

# set initial speed
v_initial = 0

# initialize vehicle body system
vehicle = sys.KinematicVehicleBody(x_initial, y_initial, yaw_initial, 
                                   v_initial)

# define the simulation time parameters
dt = 0.01
# initialize simulator system
simulator = sys.Simulator(dt)

# get initial vehicle values
x, y, yaw, v = vehicle.get_last_state()

# start simulation loop
print('started simulation...')
for idx, point in enumerate(middle_points):
    # set x and y targets as the current target point
    x_target, y_target = point
    
    # calculate the yaw target
    yaw_target = np.arctan2((y_target-y), (x_target-x))
    
    # calculate the future radius of the trajectory
    next_point = middle_points[(idx+1) % len(middle_points)]
    next_next_point = middle_points[(idx+2) % len(middle_points)]
    R = calculate_radius(point, next_point, next_next_point)
    
    # calculate the speed that maximizes lateral acceleration
    v_target = np.sqrt(MAX_ACCELERATION*R)
    
    # repeats until the vehicle is close to the target
    while not is_close(x, y, x_target, y_target):
        
        # calculate required delta
        delta = ctrl.steeringController(x, y, yaw, v, x_target, y_target,
                                        yaw_target)
        
        # calculate required acceleration
        v_dot = ctrl.accelerationController(x, y, yaw, v, x_target, y_target,
                                            yaw_target, v_target)
        
        # simulate a step with the calculated inputs
        x, y, yaw, v = simulator.simulate_step(vehicle, delta, v_dot)
        
        # checks if the simulation is taking too long
        if simulator.get_runtime() > MAX_SIMULATION_TIME:
            raise Exception('Simulation was stopped because the vehicle was '
                            'not able to complete the track. Try fixing the '
                            'MIN_DIST parameter in the constants.py file')
                  
                  
print('finished simulation.')
print('total simulated time:', np.round(simulator.get_runtime(), 3), 's')

# get simulation timestamps
t = simulator.get_timestamps()
# plot vehicle path with speeds
plots.plot_vehicle_path(vehicle, yellow_cones, blue_cones)
# plot steering and acceleration inputs
plots.plot_inputs(vehicle, t)
