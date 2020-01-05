from numpy import *
import matplotlib.pyplot as plt


# file:///D:/Wallpapers/traj.pdfBiped Robot Reference Generation with Natural
# ZMP Trajectories
# Okan Kurt and Kemalettin Erbatur
def traj(t):
    # b = .1 # half length of footsole
    # B = 5 # foot step size
    # A = 3 # d/s betwwen foot
    # T = 1
    b = .14  # half length of footsole
    B = 6  # foot step size
    A = 1  # half d/s betwwen foot
    T = 2

    g = 981
    z = 23
    w = sqrt(g / z)
    dsp = 5

    # for Cy
    sum_cy = 0
    for n in range(1, 100):
        num = (2 * A * (T ** 2) * (w ** 2) * (1 - cos(n * pi)))
        denom = (n * pi * ((T ** 2) * (w ** 2) + (n ** 2) * (pi ** 2)))

        init = num / denom

        rest = (sinc(n * pi / dsp)) * sin(n * pi * t / T)

        sum_cy += init * rest
    # for py
    sum_py = 0
    for n in range(1, 100):
        num = 2 * A * (1 - cos(n * pi))
        denom = n * pi
        init = num / denom
        rest = (sinc(n * pi / dsp)) * sin(n * pi * t / T)
        sum_py += init * rest

    # for px
    sum_px = 0
    s = (B / T) * (t - T / 2)

    for n in range(1, 100):
        num = (B - 2 * b) * (1 + cos(n * pi))
        denom = n * pi
        init = num / denom
        rest = (sinc(n * pi / dsp)) * sin(n * pi * t / T)
        sum_px += init * rest
    sum_px += s

    # for cx
    sum_cx = 0

    for n in range(1, 100):
        num = (B - 2 * b) * (1 + cos(n * pi)) * (B * T ** 2 * w ** 2)
        denom = n * pi * (T ** 2 * w ** 2 + n ** 2 * pi ** 2)
        init = num / denom
        rest = (sinc(n * pi / dsp)) * sin(n * pi * t / T)
        sum_cx += init * rest
    sum_cx += s
    return [sum_px, sum_cx, sum_cy, sum_py]

# t = arange(0,4, 0.002)
#
# plt.plot(t, traj(t)[2])
# plt.plot(t, traj(t)[3])
#
# plt.figure(2)
# plt.plot(t, traj(t)[0])
# plt.plot(t, traj(t)[1])
#
#
# plt.show()
