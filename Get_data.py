import pygame
import time
import matplotlib.pyplot as plt

pygame.init()   # initialize Window
screen = pygame.display.set_mode((1000, 500))
pygame.display.set_caption("PID Controller")
width, height = pygame.display.get_window_size()
clock = pygame.time.Clock()

pd_multiplier = 20

MouseSetpoint = width/2
x = width/2+250     # position on the x-axis
vx = 0.0    # velocity on the x-axis
ax = 0.0    # acceleration on the x-axis
i_value = 0  # I term thing
vsp = 0.0   # velocity of Setpoint used for Feed Forward Implementation
lastMouse = 0   # last position of Mouse for Feed Forward Implementation

p = 500     # p term multiplier
i = 1   # i term multiplier
d = 50  # d term multiplier
ff = 20 # Feed Forward (boosts object with setpoint derivative)
max_accel = 20000    # maximal Acceleration

wind = False  # If wind is being simulated (explaining I term)
i_enable = False

FF_enable = False # if Feed Forward is being enabled

weight = 1  # weight in kg

ms = []     # data
acc = []
vel = []
error = []
i_vals = []
baseline = []
actualVals = []

p, d = p * pd_multiplier, d * pd_multiplier     # multiplies p and d proportionally

def drawRect(pos, sp):
    pygame.draw.line(screen, (255, 255, 255), (sp, 0), (sp, height))    # draw vertical line on setpoint
    pygame.draw.rect(screen, (255, 255, 255), (pos-100/2, height/2-100/2, 100, 100))    # draw rectangle
    pygame.draw.line(screen, (127, 127, 127), (pos, height/2-100/2), (pos, height/2+100/2))     # draw a line in the middle of the rectangle


def movementUpdate(pos, velocity, acceleration, MouseSet):
    global lasttime, lastMouse
    deltaT = time.time() - lasttime
    lasttime = time.time()

    velocity += acceleration * deltaT
    if wind:
        velocity -= 1
    pos += velocity * deltaT
    setpointVelocity = (MouseSet - lastMouse)
    lastMouse = MouseSet

    return pos, velocity, setpointVelocity


def PID(setpoint, position, velocity, SetpointVel):
    global ax, p, i, d, i_value, FF_enable
    acceleration = (setpoint - position) * (p / 10)
    acceleration += (- velocity) * (d / 10)
    if -100 < setpoint - position < 100 and i_enable:
        i_value += position-setpoint
        acceleration -= i_value * (i/100)
    if FF_enable:
        acceleration += (SetpointVel) * (ff * 1000)

    return acceleration


def get_data():
    global ms, acc, vel, error, i_value
    ms.append((time.time()-starttime)*1000)
    acc.append(ax)
    vel.append(vx)
    i_vals.append(i_value)
    error.append((x - MouseSetpoint)*3)
    baseline.append((width - MouseSetpoint) * 2)
    actualVals.append((width - x) * 2)


starttime = time.time()
lasttime = time.time()

for z in range(2000):   # for linux
#for z in range(10000):    # for windows
    pygame.event.pump()     # get new mouse info

    x, vx, vsp = movementUpdate(x, vx, ax, MouseSetpoint)   # physics
    ax = PID(MouseSetpoint, x, vx, vsp)

    if ax > max_accel:   # limit max acceleration
        ax = max_accel
    elif ax < -max_accel:
        ax = -max_accel

    get_data()  # get data for plotting

    MouseSetpoint = pygame.mouse.get_pos()[0]   # save mouse info to int

    drawRect(x, MouseSetpoint)  # draw things
    pygame.display.flip()
    screen.fill((0, 0, 0))
    clock.tick()
    print(clock.get_fps())

#plt.plot(ms, acc, label="Acceleration")
#plt.plot(ms, vel, label="Velocity")
#plt.plot(ms, error, label="Error")
plt.plot(ms, i_vals, label="I-term")
plt.plot(ms, baseline, label="Setpoint")
plt.plot(ms, actualVals, label="Position")

plt.legend()
plt.show()
