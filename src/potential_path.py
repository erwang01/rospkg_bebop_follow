#!/usr/local/bin/python
""" potential_path.py
2D Potential Path Planning Algorithm
Note currently wall has yet to be implemented
Circles are under development
"""
import numpy as np

class Map:
    """ Stores all goals and obstacles """
    def __init__(self, goal=Goal(), obstacle=()):
        self.goal = goal
        self.obstacle = obstacle

class Obstacle:
    """ Obstacle Class """
    """ 2 Kinds of Obstacles: Circles and Walls) """
    """ Circles are given in radius and a (x, y) coordinate center """
    """ Walls are given in a slope and a (x, y) coordinate intercept """
    def __init__(self, kind=0, location=(0,0), d_safe=1, r=0, m=0,n=1):
        self.kind_circle=0
        self.kind_wall=1 #TODO Implement Wall
        self.kind = kind
        self.location = location
        self.d_safe = d_safe
        self.n = n
        self.r = r
        self.m = m
        self.dist = d_safe
    
    #TODO Implement Velocity
    def potential(self, location=(0,0), velocity=(0,0)):
        if self.distance(location) >= d_safe:
            return 0
        else:
            return 0.5*n*np.square(1.0/self.dist - 1.0/d_safe) #TODO is potential calculated with x,y separate or together?

    #TODO Implement Velocity
    def gradient(self, location=(0,0), velocity=(0,0)):
        if self.distance(location) >= d_safe:
            return 0
        else:
            return n*(1.0/d_safe - 1.0/self.dist)*np.square(self.dist)

    #TODO Implement Velocity
    def angle(self, location=(0,0), velocity =(0,0)):
        x = self.location[0] - location[0]
        y = self.location[1] - location[1]
        self.angle = np.arctan2(y,x)
        return angle

    #TODO Would it be more efficient if I saved the previous location and if its the same location just skip the calculation?
    def distance(location=(0,0)):
        self.dist = np.sqrt(np.sum((np.square(location[0]-self.location[0]),np.square(location[1]- self.location[1]))) - r
        return self.dist


class Goal:
    """ Goal Class """
    def __init__(self, location=(0,0), z=1, kind=0, d_target=1, d_threshold=10):
        self.kind_quadratic=0
        self.kind_conic=1
        self.kind_hybrid=2
        if kind in (0,1,2):
            self.kind = kind
        else: #TODO figure out if this is actually a ros node or just a helper class.
            rospy.logfatal("Goal kind %s is bad. Shutting down.", kind)
            rospy.signal_shutdown("Bad goal algorithm selection. Shutting down map")
        self.location = location
        self.z = z
        self.d_target = d_target
        self.d_threshold = d_threshold

    #TODO Implement Velocity
    def potential(self, location=(0,0), velocity=(0,0)):
        if self.kind == self.kind_quadratic:
            return _potential_quadratic(self, location, velocity)
        elif self.kind == self.kind_conic:
            return _potential_conic(self, location, velocity)
        else:
            if self.distance(location) > d_threshold:
                return _potential_conic(self, location, velocity)
            else:
                return _potential_quadratic(self, location, velocity)

    def _potential_quadratic(self, location, velocity):
        return 0.5*z*np.square(self.distance(location))

    def _potential_conic(self, location, velocity):
        return d_threshold*z*self.distance(location)-0.5*z*np.square(d_threshold)

   #TODO Implement Velocity
    def gradient(self, location=(0,0), velocity=(0,0)):
        if self.kind == self.kind_quadratic:
            return _gradient_quadratic(self, location, velocity)
        elif self.kind == self.kind_conic:
            return _gradient_conic(self, location, velocity)
        else:
            if self.distance(location) > d_threshold:
                return _gradient_conic(self, location, velocity)
            else:
                return _gradient_quadratic(self, location, velocity)

    def _gradient_quadratic(self, location, velocity):
        return z*self.distance(location)

    def _gradient_conic(self, location, velocity):
        return d_threshold*z #TODO figure out if this is just a constant?

    #TODO Implement Velocity
    def angle(self, location=(0,0), velocity =(0,0)):
        x = self.location[0] - location[0]
        y = self.location[1] - location[1]
        self.angle = np.arctan2(y,x)
        return angle

    #TODO Would it be more efficient if I saved the previous location and if its the same location just skip the calculation?
    def distance(location=(0,0)):
        self.dist = np.sqrt(np.sum((np.square(location[0]-self.location[0]),np.square(location[1]- self.location[1])))
        return self.dist




