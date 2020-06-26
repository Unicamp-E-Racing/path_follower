# Path Following Demonstration

## Description
Basic vehicle path following simulator with steering and acceleration control. The goal of this project is to control the steering of a simple vehicle to follow a given path. Another goal of the project is to control the vehicle's speed, accelerating or braking based on the radius of curvature of the given path.

## Software Organization

### Systems
The file systems.py contains the simulator's system objects.
#### Kinematic Vehicle Body
Single-track kinematic vehicle body model (or bicycle model) object. Stores the vehicle's states (x,y and yaw positions) and inputs (steering angle and acceleration).

![vehicle_model](https://github.com/lucasbarretto/path_follower/blob/master/style/images/bicycle_model.png "Vehicle Model")

#### Simulator
The simulator system object is responsible for solving the vehicle model's equations for a small timestep. Therefore, simulating the vehicle's state consists of iterating trough the simulator's simulation step function. Also keeps track of timestamps and total runtime.

### Controllers
The file controllers.py contains the simulator's controller functions. These controllers were designed by our team with the help of professor José C. Geromel. Both controllers receive the current vehicle state and also the position and speed targets and output the controlled variable (steering angle and acceleration).

### runTrack
The file runTrack.py simulates the steering and acceleration control of the vehicle through a complete, randomly generated, race track. The first goal of this script is to control the vehicle's steering angle, so that it passes near every middle point of the race track. The second goal is to maximize the vehicle's speed based on the radius of the track. At every track point, the radius of curvature is calculated using this formula:

![radius_formula](https://github.com/lucasbarretto/path_follower/blob/master/style/images/radius_formula.png "Radius Formula")

Then, we use the centripetal acceleration formula and the calculated radius to calculate the speed that reaches a predefined maximum lateral acceleration. Therefore, the speed of the vehicle will be increased or decreased based on the radius of the path.

#### Results
![path](https://github.com/lucasbarretto/path_follower/blob/master/style/images/runtrack_path.jpg "Path")
![inputs](https://github.com/lucasbarretto/path_follower/blob/master/style/images/runtrack_inputs.jpg "Inputs")

### Other Files
#### trackGenerator
Generates a random race track defined by yellow and blue cones, just like in the FSAE DV Trackdrive event!
#### plots
Contains the simulator's plotting functions
#### constants
Contains the simulator's constant parameter values
