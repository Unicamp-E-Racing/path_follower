### PLOTS: contains the simulator plotting functions

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from numpy import pi
import os
from constants import *

# import E-Racing team font
__location__ = os.path.realpath(os.path.join(os.getcwd(),
                                             os.path.dirname(__file__)))
fpath = os.path.join(__location__, 'style/TitilliumWeb-Regular.ttf')
prop = fm.FontProperties(fname=fpath)

# function plots the vehicle path with speeds
def plot_vehicle_path(Vehicle, yellow_cones, blue_cones):
    fig = plt.figure()
    
    # set title and labels
    plt.title('Global Vehicle Position with Speeds', fontproperties=prop,
              fontsize=16)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    
    # set background color
    ax=plt.gca()
    ax.set_facecolor('0.5')
    
    # plot vehicle position with speeds
    plt.scatter(Vehicle.x, Vehicle.y, c=Vehicle.v*3.6, cmap = 'jet', s=5)
    plt.colorbar().set_label('Speed (km/h)')
    
    # plot initial car position
    plt.scatter(Vehicle.x[0], Vehicle.y[0], color=ORANGE, s=200,
                marker=(3,0,(Vehicle.yaw[0]*180/pi-90)), edgecolor='black',
                linewidth='1')
    
    # plot race track
    plt.plot(yellow_cones[:,0], yellow_cones[:,1], '--', color='black', lw=1)
    plt.plot(blue_cones[:,0], blue_cones[:,1], '--', color='black', lw=1)
    plt.axis('equal')
    
    plt.show()
    
# plots the vehicle inputs over time
def plot_inputs(Vehicle, t):
    fig, ax = plt.subplots(2,1,sharex=True)
    fig.suptitle('Vehicle Inputs over time', fontproperties=prop,
                 fontsize=16)
    
    # plot steering input over time
    ax[0].plot(t, Vehicle.delta*180/pi, c=ORANGE, lw=2)
    ax[0].set_ylabel('Steering (degrees)')
    ax[0].grid(True)
    
    # plot input acceleration over time
    ax[1].plot(t, Vehicle.v_dot, c='black', lw=2)
    ax[1].set_ylabel('Acceleration (m/s²)')
    ax[1].set_xlabel('Time (s)')
    ax[1].grid(True)
    plt.show()