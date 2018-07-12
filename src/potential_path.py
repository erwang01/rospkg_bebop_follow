#!/usr/local/bin/python
""" potential_path.py
2D Potential Path Planning Algorithm
Note currently lines has yet to be implemented
Circles are under development
gradient values are given in (x, y)
potential values are given as U
"""
import numpy as np

def pol2cart(rho, theta):
    return (rho*np.cos(theta), rho*np.sin(theta))

#TODO update references
class ObstaclePotential:
    """ Obstacle Class """
    """ 2 Kinds of Obstacles: Circles and lines) """
    """ Circles are given in radius and a (x, y) coordinate center """
    """ lines are given in a slope and a (x, y) coordinate intercept """
    """ r is the radius of the absolute wall. """
    # TODO implement keep_out for lines as well
    """ keep_out (0,1,2):
        0 -> Keeps the object in when its inside the wall keeping it out when it is outside the wall
        1 -> forces the gradient away from the center of the obstacle even if the location is inside the wall (radius r)
        2 -> forces the gradient into the center of the obstacle even if the location is outside the wall (radius r)
    """
    def __init__(self, kind=0, location=(0,0), d_safe=1, r=0, m=0, n=1, keep_out=1, max_potential=100, max_gradient=100):
        self.kind_circle=0
        self.kind_line=1 #TODO Implement lines
        self.kind = kind
        self.location = location
        self.d_safe = d_safe
        self.n = n
        self.r = r
        self.m = m
        self.dist = d_safe
        self.keep_out_both = 0
        self.keep_out_out = 1
        self.keep_out_in = 2
        if keep_out in (0,1,2):
            self.keep_out = keep_out
        else:
            raise KeyError() #TODO figure out if this is the right error.
            self.keep_out = self.keep_out_out
        self.max_potential = max_potential
        self.max_gradient = max_gradient
    
    #TODO Implement Velocity
    def potential(self, location=(0,0), velocity=(0,0)): 
        self.distance(location)
        if self.true_dist < self.r and self.keep_out == self.keep_out_out:
            return self.max_potential #TODO make this a mountain ish thing
        elif self.true_dist > self.r and self.keep_out == self.keep_out_in:
            return self.max_potential
        elif self.dist >= self.d_safe:
            return 0
        else:
            return min(0.5*self.n*np.square(1.0/self.dist - 1.0/self.d_safe), self.max_potential)

    #TODO Implement Velocity
    def gradient(self, location=(0,0), velocity=(0,0)):
        self.distance(location)
        if self.true_dist < self.r and self.keep_out == self.keep_out_out:
            return pol2cart(-1*self.max_gradient, self.get_angle(location, velocity))
        elif self.true_dist > self.r and self.keep_out == self.keep_out_in:
            return pol2cart(self.max_gradient, self.get_angle(location, velocity))
        elif self.dist >= self.d_safe:
            return (0,0)
        else:
            return pol2cart(min(self.coeff(location)*self.n*((1.0/self.d_safe) - (1.0/self.dist))/(np.square(self.dist)),self.max_gradient), self.get_angle(location, velocity))

    #TODO Implement Velocity
    def get_angle(self, location=(0,0), velocity =(0,0)):
        x = location[0] - self.location[0]
        y = location[1] - self.location[1]
        self.angle = np.arctan2(y,x)
        return self.angle

    #TODO Would it be more efficient if I saved the previous location and if its the same location just skip the calculation?
    # consider the same thing for angle
    def distance(self, location=(0,0)):
        self.true_dist = np.sqrt(np.square(location[0]-self.location[0])+np.square(location[1]- self.location[1]))
        self.dist = np.absolute(self.true_dist-self.r)
        return self.dist
    
    def coeff(self, location=(0,0)):
        if self.keep_out == self.keep_out_out:
            return 1
        elif self.keep_out == self.keep_out_in:
            return -1
        elif self.keep_out == self.keep_out_both:
            if (self.true_dist - self.r) > 0:
                return 1
            else:
                return -1
        else:
            raise KeyError() #TODO figure out if this is the right error.
            return 0
#TODO update references
class GoalPotential:
    """ Goal Class """
    """ z: gain
        location: location of the goal
        kind: algorithm to use
        d_threshold: the threshold at which to switch between algorithms if hybrid was chosen
    """
    def __init__(self, location=(0,0), z=1, kind=0, d_threshold=10):
        self.kind_quadratic=0
        self.kind_conic=1
        self.kind_hybrid=2
        if kind in (0,1,2):
            self.kind = kind
        else:
            raise KeyError() #TODO figure out if this is the right error.
            self.kind=0
        self.location = location
        self.z = z
        self.d_threshold = d_threshold

    #TODO Implement Velocity
    def potential(self, location=(0,0), velocity=(0,0)):
        potential = 0
        if self.kind == self.kind_quadratic:
            potential = self._potential_quadratic(location, velocity)
        elif self.kind == self.kind_conic:
            potential = self._potential_conic( location, velocity)
        else:
            if self.distance(location) > self.d_threshold:
                potential = self._potential_conic(location, velocity)
            else:
                potential = self._potential_quadratic(location, velocity)
        return potential

    def _potential_quadratic(self, location, velocity):
        return 0.5*self.z*np.square(self.distance(location))

    def _potential_conic(self, location, velocity):
        return self.d_threshold*self.z*self.distance(location)-0.5*self.z*np.square(self.d_threshold)

    #TODO Implement Velocity
    def gradient(self, location=(0,0), velocity=(0,0)):
        gradient = 0
        if self.kind == self.kind_quadratic:
            gradient = self._gradient_quadratic(location, velocity)
        elif self.kind == self.kind_conic:
            gradient = self._gradient_conic(location, velocity)
        else:
            if self.distance(location=location) > self.d_threshold:
                gradient = self._gradient_conic(location, velocity)
            else:
                gradient = self._gradient_quadratic(location, velocity)
        return pol2cart(gradient, self.get_angle(location, velocity))

    def _gradient_quadratic(self, location, velocity):
        return self.z*self.distance(location)

    def _gradient_conic(self, location, velocity):
        return self.d_threshold*self.z #TODO figure out if this is just a constant?

    #TODO Implement Velocity
    def get_angle(self, location=(0,0), velocity =(0,0)):
        x = location[0] - self.location[0]
        y = location[1] - self.location[1]
        self.angle = np.arctan2(y,x)
        return self.angle

    #TODO Would it be more efficient if I saved the previous location and if its the same location just skip the calculation?
    def distance(self, location=(0,0)):
        self.dist = np.sqrt(np.square(location[0]-self.location[0])+np.square(location[1]- self.location[1]))
        return self.dist

    def set_location(self, location): #TODO check if location is valid.
        self.location = location

#TODO update references
class MapPotential:
    """ Stores all goals and obstacles """
    def __init__(self, goal=GoalPotential(), obstacles=()):
        self.goal = goal
        self.obstacles = obstacles

    #TODO Implement velocity
    def potential(self, location, velocity=(0,0)):
        potential_obstacles = 0
        for obstacle in self.obstacles:
            potential_obstacles += obstacle.potential(location, velocity)
        return self.goal.potential(location, velocity)+potential_obstacles

    #TODO Implement velocity
    # Returns the negative gradient such that it can be directly inputted into speed.
    def gradient(self, location, velocity=(0,0)):
        gradient_obstacles = (0,0)
        for obstacle in self.obstacles:
            gradient_obstacles = np.add(gradient_obstacles, obstacle.gradient(location, velocity))
        return -1*np.add(self.goal.gradient(location, velocity), gradient_obstacles)

    def get_goal(self):
        return self.goal

    def set_goal(self, goal): #TODO verify goal
        self.goal = goal
