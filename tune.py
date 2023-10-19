import matplotlib.pyplot as plt
import numpy as np

x = []
y = []

errors = [0]

pos = 0
goal = 90

v = 0
a = 0

# PID constants
kP = 0.1
kI = 0
kD = 0.1

# Arm Constants
kStatic = 0.22
kGravity = 0.23
kVelocity = 1.5
kAcceleration = 0.03

ff = 0

t = 0
while( t < 100):
    t += 0.02

    error = goal - pos
    errors.append(error)

    p = kP * error
    i = kI * sum(errors)
    d = kD * (error - errors[len(errors) - 2])

    ff = 0 # goal * (kVelocity * v + kAcceleration * a)

    appliedVolts = max(min(ff + p + i + d, 12), -12)

    a = (-kVelocity / kAcceleration)*v + (appliedVolts / kAcceleration)
    v += a
    pos += v

    x.append(t)
    y.append(pos)

xpoints = np.array(x)
ypoints = np.array(y)

plt.xlabel('Time')
plt.ylabel('Position')
plt.title('PID Tuning')
plt.plot(xpoints, ypoints)
plt.show()
