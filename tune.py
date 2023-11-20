import matplotlib.pyplot as plt
import numpy as np
import math

from linearSim import LinearSystemSim
from linearSystem import LinearSystem

x = []
y = []

errors = [0]
appliedVoltsList = []

pos = 0
goal = 90

v = 0
a = 0

# PID constants
kP = 0
kI = 0
kD = 0

# Arm Constants
kStatic = 0.22
kGravity = 0.23
kVelocity = 1.5
kAcceleration = 0.03

ff = 0
p = 0.1
i = 0
d = 0

def calculateFeedForward(v, a):
    return kStatic * np.sign(v) + kVelocity * v + kAcceleration * a

system = LinearSystem(np.array([[-kVelocity/kAcceleration]]), np.array([[1/kAcceleration]]), np.array([[1]]), np.array([[0]]))
sim = LinearSystemSim(system, measurement_std_devs=None)

dt = 0.02
t = 0
while( t < 100):
    t += dt
    sim.update(dt)

    error = goal - pos
    errors.append(error)

    p = kP * error
    i = kI * sum(errors)
    d = kD * (error - errors[len(errors) - 2])

    ff = calculateFeedForward(v, a)

    appliedVolts = max(min(p+i+d, 12), -12)
    sim.set_input(appliedVolts)

    v = sim.get_output_element(0)
    pos += v

    x.append(t)
    y.append(pos)

print("Final Position: ", pos)
# print("Errors: ", errors)

xpoints = np.array(x)
ypoints = np.array(y)
plt.ylabel('Position')
plt.title('PID Tuning')
plt.plot(xpoints, ypoints)
plt.show()
