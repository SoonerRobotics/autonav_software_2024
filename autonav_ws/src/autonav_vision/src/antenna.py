import pygame
import random

pygame.init()

MAX_LENGTH = 100

# colors
WHITE = (255, 255, 255)

# normal pygame initialization code
WIDTH = 640
HEIGHT = 480
flags = 0 #pygame.OPENGL
screen = pygame.display.set_mode((WIDTH, HEIGHT), vsync=1, flags=flags)

clock = pygame.time.Clock()

class Feeler:
    def __init__(self, max_length, angle):
        self.max_length = length
        self.angle = angle
        self.length = max_length
    
    def get_output(self):
        return self.max_length - self.length

class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.vel = 0
        self.heading = 0

        self.feelers = [Feeler(MAX_LENGTH, angle) for angle in range(0, 360, 10)]
    
    def update_feelers(self):
        for feeler in self.feelers:
            feeler.update() #TODO



# pygame doesn't have a circle class? makes sense, though, honestly
class Circle():
    def __init__(self, x, y, radius, color) :
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
    
    def draw(self, surface, color=None):
        if color is None:
            pygame.draw.circle(surface, self.color, (self.x, self.y), self.radius)
        else:
            pygame.draw.circle(surface, color, (self.x, self.y), self.radius)


# main loop
while True:
    for event in pygame.event.get():
        # quit on either pressing Q or hitting the X on the window
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
            pygame.quit()
            #opencv.destroyAllWindows() # just in case, y'know? 'cause otherwise we're gonna have some open windows. and the wind's pretty strong
            raise SystemExit

        # handle all the different keypresses we're binding stuff to
        elif event.type == pygame.KEYUP: # on keyup so it only triggers once per keypress and we don't have to debounce stuff and do weird bool stuff and whatnot
            pass
            # if event.key == pygame.K_r: #(r)egenerate all clouds
            #     clouds = make_clouds()

            # # almost made resume a separate key (was thinking spacebar) but I think toggling pause is much cleaner and more intuitive
            # elif event.key == pygame.K_p: # toggle (p)ause for all clouds, mostly for debugging and whatnot
            #     # wait what if pausing was on a per-cloud basis? so in the future we could make custom clouds that pause/stop on their own? I like that better yeah
            #     for cloud in clouds:
            #         cloud.pause = not cloud.pause
            

    screen.fill(WHITE)


    # paint all the clouds (after the sun, because they need to pass between us and the sun, because physically that's how clouds work, obviously)
    for cloud in clouds:
        cloud.draw()
        cloud.update()

    pygame.display.flip()

    clock.tick(120) # that's a little spicy fast gonna be honest