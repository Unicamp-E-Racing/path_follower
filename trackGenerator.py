### RANDOM RACE TRACK GENERATOR
# PARAMETERS:
#  - num_points: [min max] number of points generated
#  - max_point_range: max range of a generated point
#  - difficulty: 0 to 1 specifies the displacement of the track's curves
#  - max_disp: max displacement of generated points
#  - cone_dist: distance between two cones in a set
#  - set_dist: distance between two sets of cones
#  - R: cleaning radius
#
# OUTPUTS:
#  - sup_cones: [x;y] position of the superior line cones
#  - inf_cones: [x;y] position of the inferior line cones

import random
from scipy.spatial import ConvexHull
import numpy as np
import math

# removes data points that are too close to each other
def clean(data, R):
    clean_data = np.array([[0,0]])

    for i in range(len(data)-1):
        flag = 0

        for j in range(i+1, len(data)):
            # if points are closer than R, remove them from the array
            if math.sqrt((data[i,0]-data[j,0])**2 + (data[i,1]-data[j,1])**2) < R:
                flag = 1

        if flag == 0:
            clean_data = np.concatenate((clean_data, np.array([data[i,:]])))

    clean_data = np.concatenate((clean_data, np.array([data[i+1,:]])))

    return clean_data[1:len(clean_data),:]


# calculates the Catmull Rom spline for a set of control points
def CatmullRomSpline(P0, P1, P2, P3, nPoints=100):

    # Convert the points to numpy so that we can do array multiplication
    P0, P1, P2, P3 = map(np.array, [P0, P1, P2, P3])

    # Calculate t0 to t4
    alpha = 0.5
    def tj(ti, Pi, Pj):
        xi, yi = Pi
        xj, yj = Pj
        return (((xj-xi)**2 + (yj-yi)**2 )**0.5 )**alpha + ti

    t0 = 0
    t1 = tj(t0, P0, P1)
    t2 = tj(t1, P1, P2)
    t3 = tj(t2, P2, P3)

    # Only calculate points between P1 and P2
    t = np.linspace(t1,t2,nPoints)

    # Reshape so that we can multiply by the points P0 to P3
    # and get a point for each value of t.
    t = t.reshape(len(t),1)

    A1 = (t1-t)/(t1-t0)*P0 + (t-t0)/(t1-t0)*P1
    A2 = (t2-t)/(t2-t1)*P1 + (t-t1)/(t2-t1)*P2
    A3 = (t3-t)/(t3-t2)*P2 + (t-t2)/(t3-t2)*P3
    B1 = (t2-t)/(t2-t0)*A1 + (t-t0)/(t2-t0)*A2
    B2 = (t3-t)/(t3-t1)*A2 + (t-t1)/(t3-t1)*A3

    C  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2
    return C

# calculates Catmull Rom for a chain of points and return the combined curve.
def CatmullRomChain(P):
    sz = len(P)

    # The curve C will contain an array of (x,y) points.
    C = []
    for i in range(sz-3):
        c = CatmullRomSpline(P[i], P[i+1], P[i+2], P[i+3])
        C.extend(c)

    return C

# returns an ndarray containing the normalized gradients of the track
def getGrad(track):
    grad = np.stack(np.gradient(track, axis=0))
    norm = np.vstack(np.linalg.norm(grad, axis=1))

    for i in range(len(grad)):
        grad[i] = grad[i]/norm[i]

    return grad

# generates the cone positions of the outer and inner track lines
def genTrack(max_point_range, difficulty, max_disp, cone_dist, set_dist, point_count, R):
    point_count = random.randint(point_count[0], point_count[1])

    # generates centered points in a rectangle
    points_x = np.array([])
    points_y = np.array([])

    for i in range(point_count):
        points_x = np.append(points_x, random.uniform(0, max_point_range))
        points_y = np.append(points_y, random.uniform(0, max_point_range))

    points = np.concatenate(([points_x], [points_y]), axis=0)

    # uses the convex hull algorythm to generate a polygon
    hull = ConvexHull(points.T)
    data = points[:, hull.vertices].T

    # generates new displaced points in between the existing ones
    for i in range(len(data)):

        # generates point with random length and angle
        disp_len = random.random()**difficulty * max_disp
        disp = np.concatenate(([np.array([0])], [np.array([disp_len])]), axis=0)
        theta = 2*math.pi*random.random()
        rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                               [np.sin(theta), np.cos(theta)]])
        disp = np.matmul(rot_matrix,disp)

        if i == 0:
            r_set = np.array([data[0,:]])
        else:
            r_set = np.concatenate((r_set, np.array([data[i,:]])), axis=0)

        # apply displacement to each point
        if i == len(data)-1:
            a = (data[i,:] + data[0,:])/2 + disp.T
        else:
            a = (data[i,:] + data[i+1,:])/2 + disp.T

        r_set = np.concatenate((r_set,a), axis=0)

    # removes points that are too close to each other
    data = clean(r_set, R);

    # prepares data for Catmull Rom
    data = np.concatenate((data, np.array([data[0,:]])))
    data = np.concatenate((data, np.array([data[1,:]])))
    data = np.concatenate((data, np.array([data[2,:]])))

    # apllies the Catmull Rom spline to the points
    track = np.vstack(CatmullRomChain(data))
    
    # removes the initial point from the end of the array
    track = track[0:len(track)-1]

    # calculates the normalized gradient for each point of the track
    grad = getGrad(track)

    # generates the inferior and superior cones
    inner_x = np.vstack(track[0:len(track):set_dist,0]) - cone_dist*np.vstack(grad[0:len(track):set_dist,1])
    inner_y = np.vstack(track[0:len(track):set_dist,1]) + cone_dist*np.vstack(grad[1:len(track):set_dist,0])
    innerCones = np.concatenate((inner_x, inner_y), axis=1)

    outer_x = np.vstack(track[0:len(track):set_dist,0]) + cone_dist*np.vstack(grad[1:len(track):set_dist,1])
    outer_y = np.vstack(track[0:len(track):set_dist,1]) - cone_dist*np.vstack(grad[0:len(track):set_dist,0])
    outerCones = np.concatenate((outer_x, outer_y), axis=1)

    return innerCones, outerCones

# generates a random race track with pre-defined input values
def genSimpleTrack():
    # parameter definition
    point_count = [10,20]
    max_point_range = 100
    max_disp = 7
    difficulty = 1
    R = 10
    set_dist = 30
    cone_dist = 3
    
    # track generation
    innerCones, outerCones = genTrack(max_point_range, difficulty, max_disp,
                                      cone_dist, set_dist, point_count, R)
    
    return innerCones, outerCones