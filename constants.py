### CONSTANTS: this file contains all the simulator constants

from numpy import pi

Lb = 1.5 # Wheelbase distance (m)
INITIAL_SPEED = 1 # initial vehicle speed for numerical stability
MAX_DELTA = pi/6 # max steering angle (rad)
MAX_ACCELERATION = 5 # max vehicle acceleration (m/s^2)
MIN_DIST = 3 # min distance between the vehicle and the target point (m)
MAX_RADIUS = 1000 # max radius of trajectory (m)
MAX_SIMULATION_TIME = 100 # max allowed simulation execution time (s)
ORANGE = '#FF6600' # E-Racing's orange color hex code