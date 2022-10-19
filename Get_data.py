import pygame
import time
import matplotlib.pyplot as plt

pygame.init()   # initialize Window
screen = pygame.display.set_mode((1000, 500))
pygame.display.set_caption("PID Controller")
width, height = pygame.display.get_window_size()

MouseSetpoint = width/2
x = width/2+250     # position on the x-axis
vx = 0.0    # velocity on the x-axis
ax = 0.0    # acceleration on the x-axis
i_value = 0  # I term thing

p = 200     # p term multiplier
i = 1   # i term multiplier
d = 50  # d term multiplier
ff = 50 # Feed Forward (boosts object with setpoint derivative)

wind = False  # If wind is being simulated (explaining I term)
i_enable = False

FF_enable = True # if Feed Forward is being enabled

weight = 1  # weight in kg

ms = []     # data
acc = []
vel = []
error = []
i_vals = []
baseline = []
actualVals = []


def drawRect(pos, sp):
    pygame.draw.line(screen, (255, 255, 255), (sp, 0), (sp, height))    # draw vertical line on setpoint
    pygame.draw.rect(screen, (255, 255, 255), (pos-100/2, height/2-100/2, 100, 100))    # draw rectangle
    pygame.draw.line(screen, (127, 127, 127), (pos, height/2-100/2), (pos, height/2+100/2))     # draw a line in the middle of the rectangle


def movementUpdate(pos, velocity, acceleration):
    global lasttime
    deltaT = time.time() - lasttime
    lasttime = time.time()

    velocity += acceleration * deltaT
    if wind:
        velocity -= 1
    pos += velocity * deltaT

    return pos, velocity


def PID(setpoint, position, velocity):
    global ax, p, i, d, i_value
    acceleration = (setpoint - position) * (p / 10)
    acceleration += (- velocity) * (d / 10)
    if -100 < setpoint - position < 100 and i_enable:
        i_value += position-setpoint
        acceleration -= i_value * (i/100)

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
    print(i_value)


starttime = time.time()
lasttime = time.time()

# for z in range(500):   # for linux
for z in range(10000):    # for windows
    pygame.event.pump()     # get new mouse info

    x, vx = movementUpdate(x, vx, ax)   # physics
    ax = PID(MouseSetpoint, x, vx)

    if ax > 3000:   # limit max acceleration
        ax = 3000
    elif ax < -3000:
        ax = -3000

    get_data()  # get data for plotting

    MouseSetpoint = pygame.mouse.get_pos()[0]   # save mouse info to int

    drawRect(x, MouseSetpoint)  # draw things
    pygame.display.flip()
    screen.fill((0, 0, 0))

#plt.plot(ms, acc, label="Acceleration")
#plt.plot(ms, vel, label="Velocity")
#plt.plot(ms, error, label="Error")
plt.plot(ms, i_vals, label="I-term")
plt.plot(ms, baseline, label="Setpoint")
plt.plot(ms, actualVals, label="Position")

plt.legend()
plt.show()
