import pygame as pg
from seaborn import color_palette
import numpy as np
import helpers
from boid import Boid

def main():
    # Parameters controlling the simulation (these are default, based on https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html)
    N = 100 # Number of boids created using random initialization
    SEED = None # Seed for random settings
    INITIALIZE = True # If true, randomly initializes N boids at the beginning of simulation

    FPS = 25 # Frames per second in the pygame window - how many updates of the system will be in one second
    FOV = 180 # Boid's field of view given in degrees 
    ROV = 40 # Boid's range of view - how far can it see
    PROTECTED_RANGE = 12 # If another object will be within this range from a boid, it will try to avoid it
    MAX_SPEED = 200.0 # Maximal speed of a boid (pixels per second)
    MIN_SPEED = 100.0 # Minimal speed of a boid (pixels per second)
    AVOIDING = 0.05 # Influence of the avoiding behaviour on the movement (change with caution)
    CENTERING = 0.0005 # Influence of the centering behaviour on the movement (change with caution)
    MATCHING = 0.05 # Influence of the matching behaviour on the movement (change with caution)
    TURNING = 0.4 # Rapidity of turning at borders (keep between 0 and 1 for the best results)
    L = 1000 # Size of the system (in pixels)
    MARGIN = L/10 # Size of the system's margin - area in which boids are turning

    BOID_SIZE = L/200 # Size of a single boid

    parameters = {
        'rov': ROV,
        'protected_range': PROTECTED_RANGE,
        'max_speed': MAX_SPEED,
        'min_speed': MIN_SPEED,
        'avoid': AVOIDING,
        'centering': CENTERING,
        'matching': MATCHING,
        'turn': TURNING * MIN_SPEED / 2,
        'L': L,
        'margin': MARGIN,
        'fov': np.radians(FOV)
    }

    np.random.seed(SEED)
    
    # Working with colors
    cp = color_palette('hsv', 360) # Color palette for boids - color of the boid changes depending on its direction (one per degree unit)
    
    # Creating the simulation
    pg.init()
    gameDisplay = pg.display.set_mode((L,L))
    pg.display.set_caption("Boid model")
    #gameDisplay.fill((0,0,0))
    clock = pg.time.Clock()

    if INITIALIZE:
        boids = [Boid(np.random.rand(2)*L*0.8+0.1,np.random.rand(2)-0.5, parameters) for _ in range(N)]
    else:
        boids = [] # List for boids

    def update():
        """
        Single update of the simulation
        """
        dt = clock.tick(FPS) / 1000  # Time step
        gameDisplay.fill((0,0,0)) # Clearing the background
        tmp = boids.copy() # A copy of the boids list for a synchronous updating - all boids are updated at the same time
        
        for b in boids:
            b.move(tmp, dt) # Move the boid
            color = helpers.float_to_uint8(cp[int(np.rad2deg(b.vangle))]) # Get a color based on its direction

            pg.draw.circle(gameDisplay, 
                           pg.Color(*color), 
                           (b.x, b.y), 
                           BOID_SIZE) # Draw the body
            pg.draw.line(gameDisplay, 
                         pg.Color(*color), 
                         (b.x,b.y), (b.x + b.vx / b.speed * BOID_SIZE * 2, b.y + b.vy / b.speed * BOID_SIZE * 2),
                         2) # Draw the line pointing in the direction of the boid

    
    # Main simulation    
    from_pos = (-1, -1) # Mouse position
    while True:
        events = pg.event.get()
        for event in events:
            if event.type == pg.QUIT: # Quitting the simulation
                pg.quit()           
                quit()
            elif event.type == pg.MOUSEBUTTONDOWN: # Get mouse position at the beginning of click
                from_pos = pg.mouse.get_pos()
            elif event.type == pg.MOUSEBUTTONUP: # Releasing the mouse button
                if event.button == 1: # For LMB create a new boid
                    to_pos = pg.mouse.get_pos()
                    if from_pos == to_pos or from_pos == (-1, -1): # For a quick click, create a boid with random direction
                        vel = np.random.rand(2) - .5
                    else: # If the mouse was moved with the LPM pressed, the boid will go in the direction of movement
                        vel = [-from_pos[0] + to_pos[0], -from_pos[1] + to_pos[1]]
                    boids.append(Boid(from_pos, vel, parameters))
                    from_pos = (-1, -1)
                else: # FOR RMB remove the most recent boid
                    if len(boids) > 0:
                        boids.pop()

        update()
        pg.display.update()

if __name__ == "__main__":
    main()