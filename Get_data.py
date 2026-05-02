import pygame
import time
import matplotlib.pyplot as plt

pygame.init()   # initialize Window
screen = pygame.display.set_mode((1000, 500))
pygame.display.set_caption("PID Controller")
width, height = pygame.display.get_window_size()
clock = pygame.time.Clock()

sim_speed:int = 10
sim_step:int = 0
dt:float = 1/sim_speed

MouseSetpoint:float = width/2
x:float = width/2+250     # position on the x-axis
vx:float = 0.0    # velocity on the x-axis
ax:float = 0.0    # acceleration on the x-axis
i_value:float = 0  # I term thing

master_scaling:float = 10 # scales every term the same way
p:float = 5    # p term multiplier
i:float = 1   # i term multiplier
d:float = 24   # d term multiplier
ff = 50 # Feed Forward (boosts object with setpoint derivative)

p /= master_scaling
i /= master_scaling
d /= master_scaling
ff /= master_scaling

wind:bool = False  # If wind is being simulated (explaining I term)
i_enable:bool = False
correct_d:bool = False

last_error:float = 0.0

FF_enable = True # if Feed Forward is being enabled

collect_ms = []     # data
collect_acc = []
collect_vel = []
collect_error = []
collect_i_vals = []
collect_baseline = []
collect_actualVals = []


def drawRect(pos, sp):
    pygame.draw.line(screen, (255, 255, 255), (sp, 0), (sp, height))    # draw vertical line on setpoint
    pygame.draw.rect(screen, (255, 255, 255), (pos-100/2, height/2-100/2, 100, 100))    # draw rectangle
    pygame.draw.line(screen, (127, 127, 127), (pos, height/2-100/2), (pos, height/2+100/2))     # draw a line in the middle of the rectangle


def movementUpdate(pos, velocity, acceleration):
    global dt
    velocity += acceleration * dt
    velocity *= 0.98 ** dt    # add a bit of friction
    if wind:
        velocity -= 1 * dt
    pos += velocity * dt

    return pos, velocity


def PID(setpoint, position, velocity):
    global ax, p, i, d, i_value, last_error, correct_d, dt
    error:float = setpoint - position
    acceleration = error * p # p
    if correct_d:
        acceleration += (error - last_error) / dt * d # d
        last_error = error
    else:
        acceleration += -velocity * d
    if i_enable:
        i_value += error * dt
        acceleration += i_value * i   # i

    return acceleration


def collect_data():
    global collect_ms, collect_acc, collect_vel, collect_error, i_value, sim_step
    collect_ms.append(sim_step)
    sim_step += 1
    collect_acc.append(ax)
    collect_vel.append(vx)
    collect_i_vals.append(i_value / 100.0)
    collect_error.append((x - MouseSetpoint) * 3)
    collect_baseline.append((width - MouseSetpoint) * 2)
    collect_actualVals.append((width - x) * 2)

# wait until mouse button is pressed
while 1:
    pygame.event.pump()

    if pygame.mouse.get_pressed()[0]:
        break

    MouseSetpoint = pygame.mouse.get_pos()[0]   # save mouse info to int

    drawRect(x, MouseSetpoint)  # draw things
    pygame.display.flip()
    screen.fill((0, 0, 0))

starttime = time.time()
# for z in range(500):   # for linux
for z in range(120):    # for windows

# UI
    clock.tick(60)
    pygame.event.pump()     # get new mouse info

    MouseSetpoint = pygame.mouse.get_pos()[0]   # save mouse info to int

    drawRect(x, MouseSetpoint)  # draw things
    pygame.display.flip()
    screen.fill((0, 0, 0))

# Physics

    for f in range(sim_speed):
        x, vx = movementUpdate(x, vx, ax)   # physics
        ax = PID(MouseSetpoint, x, vx)

        ax = max(min(ax, 50 * dt), -50 * dt)

# Data collection

    collect_data()  # collect data for plotting


pygame.quit()

#plt.plot(collect_ms, collect_acc, label="Acceleration")
#plt.plot(collect_ms, collect_vel, label="Velocity")
#plt.plot(collect_ms, collect_error, label="Error")
#plt.plot(collect_ms, collect_i_vals, label="I-term")
plt.plot(collect_ms, collect_baseline, label="Setpoint")
plt.plot(collect_ms, collect_actualVals, label="Position")

plt.legend()
plt.show()
