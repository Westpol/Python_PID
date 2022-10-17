import pygame
import time

pygame.init()
screen = pygame.display.set_mode((1000, 500))
pygame.display.set_caption("PID Controller")
width, height = pygame.display.get_window_size()

MouseSetpoint = width/2
x = width/2+250     # position on the x-axis
vx = 0.0    # velocity on the x-axis
ax = 0.0    # acceleration on the x-axis

p = 200     # p term multiplier
i = 1   # i term multiplier
d = 80  # d term multiplier

weight = 1  # weight in kg


def drawRect(pos, sp):
    pygame.draw.line(screen, (255, 255, 255), (sp, 0), (sp, height))
    pygame.draw.rect(screen, (255, 255, 255), (pos-100/2, height/2-100/2, 100, 100))
    pygame.draw.line(screen, (127, 127, 127), (pos, height/2-100/2), (pos, height/2+100/2))


def movementUpdate(pos, velocity, acceleration):
    global lasttime
    deltaT = time.time() - lasttime
    lasttime = time.time()

    velocity += acceleration * deltaT
    pos += velocity * deltaT

    return pos, velocity


def PID(setpoint, position, velocity):
    global ax, p, i, d
    acceleration = (setpoint - position) * (p / 10)
    acceleration += (- velocity) * (d / 10)
    return acceleration


lasttime = time.time()
while 1:
    pygame.event.pump()
    x, vx = movementUpdate(x, vx, ax)
    ax = PID(MouseSetpoint, x, vx)
    if ax > 3000:
        ax = 3000
    elif ax < -3000:
        ax = -3000
    MouseSetpoint = pygame.mouse.get_pos()[0]
    print(MouseSetpoint)
    drawRect(x, MouseSetpoint)
    print("Pos: {0} | Vel: {1} | Acc: {2}".format(x, vx, ax))
    pygame.display.flip()
    screen.fill((0, 0, 0))
