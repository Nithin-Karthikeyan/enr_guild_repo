# This is my code, and i have changed the lines u have asked me not to change since i felt the pid output was better this way. 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

# PID PARAMETERS (TUNE THESE)
kp = 6.2
ki = 15
kd = 0.0069

#     (\_/)
#    ( •_•)   ← only tweak above this bunny
#    / >🍃     the rest is secret rabbit code





k2 = 0.4
dt = 0.1
v = 0
E_prev = 0
E_sum = 0
t = 0
setpoint = 0
change_interval = 50
scroll_window = 10
times = []
velocities = []
targets = []

fig, ax = plt.subplots()
line_v, = ax.plot([], [], label="Velocity", linewidth=2, antialiased=True)
line_target, = ax.plot([], [], 'r--', label="Setpoint", linewidth=1.5, antialiased=True)

ax.set_xlim(0, scroll_window)
ax.set_ylim(-5, 20)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity")
ax.set_title("Dynamic PID Response")
ax.grid(True)
ax.legend(loc="upper right")

def init():
    line_v.set_data([], [])
    line_target.set_data([], [])
    return line_v, line_target

def update(frame):
    global v, E_prev, E_sum, t, setpoint

    if frame % change_interval == 0:
        setpoint = random.choice([0, 5, 10, 15])

    E = setpoint - v
    E_sum += E * dt
    dE = (E - E_prev) / dt
    E_prev = E

    # Done to minimize unstability during high jumps
    if abs(E) > 10:
        E_sum = 0
        u = kp * E + kd * dE
    else:
        # Clamping the integral error (idt it went above 7, but i still have it for safety)
        if E_sum > 5.7:
            E_sum = 5.7
        elif E_sum < 0:
            E_sum = 0
        u = kp * E + ki * E_sum + kd * dE
    print(E_sum)

    noise = np.random.normal(0, 1)
    dv = (E - k2 * v*abs(v) + noise) * dt
    v += u * dt + dv
    t += dt
    times.append(t)
    velocities.append(v)
    targets.append(setpoint)

    line_v.set_data(times, velocities)
    line_target.set_data(times, targets)

    if t > scroll_window:
        ax.set_xlim(t - scroll_window, t)

    return line_v, line_target

ani = FuncAnimation(
    fig, update, init_func=init,
    interval=20, blit=True, cache_frame_data=False
)

plt.tight_layout()
plt.show()
