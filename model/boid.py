from numpy.random import rand
from numpy import pi as PI
import helpers

class Boid:
    """
    Single element of the simulation.
    """
    def __init__(self, position: list[float], velocity: list[float], parameters: dict): 
        """
        Creates an instance of the Boid.

        ### Arguments
            - `position` - vector of coordinates in 2D space, initial position of the boid
            - `velocity` - vector of velocity in 2D space, initial velotcity of the boid
            - `parameters` - dictionary of the simulation parameters. Must include `'max_speed'`, `'min_speed'`,
                `'fov'`, `'rov'`, `'protected_range'`, `'matching'`, `'avoid'`, `'centering'`, `'margin'`, `'L'`, `'turn'`. 
        """

        self.param = parameters

        # Initialize position
        self.x = position[0]
        self.y = position[1]
        
        # Initial length is randomized between max and min speed
        init_speed = rand() * (parameters['max_speed'] - parameters['min_speed']) + parameters['min_speed']
        self.vx = velocity[0] / helpers.distance(velocity) * init_speed
        self.vy = velocity[1] / helpers.distance(velocity) * init_speed
        self.speed = init_speed

        # Angle of velocity vector
        self.vangle = helpers.angle([0, 0], velocity)

        # Field of view range - minimal and maximal at which the boid can see other boids
        # We construct it by rotating the original range [-FOV/2, FOV/2] (where FOV is the whole angle of the field) 
        # by the angle of velocity vector (direction of the boid) 
        self.fovRange = [-parameters['fov'] / 2, parameters['fov'] / 2]
        self.__rotate_fov()


    def __rotate_fov(self):
        """
        Rotate the boid's field of view to match the direction of the velocity
        """
        # If the new angle is larger than pi rad, we shift it to the other side (from -pi rad)
        if self.fovRange[0] + self.vangle > PI:
            a1 = self.fovRange[0] + self.vangle - 2 * PI
        # Similarily for angle smaller than -pi rad
        elif self.fovRange[0] + self.vangle < - PI:
            a1 = self.fovRange[0] + self.vangle + 2 * PI
        # In any other case we just shift it
        else:
            a1 = self.fovRange[0] + self.vangle

        # The same procedure for 2nd bound of the range
        if self.fovRange[1] + self.vangle > PI:
            a2 = self.fovRange[1] + self.vangle - 2 * PI
        elif self.fovRange[1] + self.vangle < -PI:
            a2 = self.fovRange[1] + self.vangle + 2 * PI
        else:
            a2 = self.fovRange[1] + self.vangle

        self.fovRange = [a1, a2]

    
    def has_in_fov(self, other):
        """
        Check if the `other` instance of Boid class is in the field of view of this Boid.
        """

        # Positions of both boids
        pos_self = [self.x, self.y]
        pos_other = [other.x, other.y]

        # Checking the angle (if it is in the fovRange). Because of the jump line from -pi to pi we consider two cases
        # 1st: The second bound is larger than the first, meaning that the fovRange doesn't cross the jump line
        #      then we just check if angle is between this ranges
        # 2nd: The second bound is smaller than the first, meaning the fovRange crosses the jump line
        #      so the angle is in fovRange if it larger than 2n bound or smaller than 1st bound
        isInFovRange = lambda i: i <= self.fovRange[1] and i >= self.fovRange[0] \
            if self.fovRange[1] > self.fovRange[0] \
            else i <= self.fovRange[1] or i >= self.fovRange[0]

        r = helpers.distance(pos_self, pos_other) # Distance between boids
        a = helpers.angle(pos_self, pos_other) # Angle between boids
        return r <= self.param['rov'] and r > 0 and isInFovRange(a) # If distance is smalled than radius of view and angle is in range of FOV, then boid is in FOV

    
    def __normalize_speed(self):
        """
        Normalizes the speed of the boid so it was within the `[min_speed, max_speed]` range.
        """
        # Compute current velocity vector length
        speed_new = helpers.distance([self.vx, self.vy])
        self.speed = speed_new
        
        # If it exceeds the maximal speed, renormalize it to maximal speed
        if speed_new > self.param['max_speed']:
            self.vx = self.vx / speed_new * self.param['max_speed']
            self.vy = self.vy / speed_new * self.param['max_speed']
            self.speed = self.param['max_speed']

        # If it is smaller than the minimal speed, renormalize it to the minimal speed
        if speed_new < self.param['min_speed']:
            self.vx = self.vx / speed_new * self.param['min_speed']
            self.vy = self.vy / speed_new * self.param['min_speed']
            self.speed = self.param['min_speed']

        # Update the new velocity angle and field of view
        self.vangle = helpers.angle([0, 0], [self.vx, self.vy])
        self.__rotate_fov()

   
    def __update_position(self, time_step=1):
        """
        Updates the position of the boid with the specified time step `time`
        """
        # To current position add the velocity multiplied by the time step (default 1)
        self.x = self.x + self.vx * time_step
        self.y = self.y + self.vy * time_step

    
    def move(self, others, time_step = 1):
        """
        Make the boid move in the space depending on the list of other boids in the simulation `others` and with some time step `dt`
        """
        # Initializing the variables
        # Distances from too close boids
        close_dx = 0
        close_dy = 0
        # Average velocity of neighbors
        xvel_avg = 0
        yvel_avg = 0
        # Average position of neighbors
        xpos_avg = 0
        ypos_avg = 0
        # Number of neighbors in FOV
        neighbors_ctr = 0

        # Iterate over other boids
        for other in others:
            # If the other boid is in FOV 
            if self.has_in_fov(other):
                neighbors_ctr += 1 # Increment the counter

                # Separation - if distance between boids is smaller than the protected range
                if helpers.distance([self.x, self.y], [other.x, other.y]) <= self.param['protected_range'] :
                    close_dx += self.x - other.x
                    close_dy += self.y - other.y
                
                # Alignment 
                xvel_avg += other.vx
                yvel_avg += other.vy

                # Cohesion
                xpos_avg += other.x
                ypos_avg += other.y

        # Updating velocity
        # Change the velocity so that the boid will move in the direction away from too close neighbors
        self.vx += close_dx * self.param['avoid'] 
        self.vy += close_dy * self.param['avoid']

        # Change the velocity so that the boid will move in the same direction as neighbors and towards the mass centre of its neighbors
        if (neighbors_ctr > 0):
            self.vx += (xvel_avg / neighbors_ctr - self.vx) * self.param['matching'] \
                    + (xpos_avg / neighbors_ctr - self.x) * self.param['centering']

            self.vy += (yvel_avg / neighbors_ctr - self.vy) * self.param['matching'] \
                    + (ypos_avg / neighbors_ctr - self.y) * self.param['centering']

        # Turning at borders - change the velocity so that the boid will turn back if it reaches the specified margin
        if (self.x <= self.param['margin']): # Left margin
            self.vx += self.param['turn']
        if (self.x >= self.param['L'] - self.param['margin']): # Right margin
            self.vx -= self.param['turn']
        if (self.y <= self.param['margin']): # Bottom margin
            self.vy += self.param['turn']
        if (self.y >= self.param['L'] - self.param['margin']): # Top margin
            self.vy -= self.param['turn']

            
        # Normalizing the velocity vector
        self.__normalize_speed()
                
        # Updating the position
        self.__update_position(time_step)

        # Forcing the boid to stay within the area of simulation
        if (self.x <= 0): # Left margin
            self.x = 0
        if (self.x >= self.param['L']): # Right margin
            self.x = self.param['L']
        if (self.y <= 0): # Bottom margin
            self.y = 0
        if (self.y >= self.param['L']): # Top margin
            self.y = self.param['L']