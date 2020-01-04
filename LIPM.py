from numpy import *
import matplotlib.pyplot as plt


def LIPM(speed, T_supp, zc):
    t = linspace(0, T_supp, T_supp / speed)

    g = 9.810

    # initial condition
    xi = 0
    vi_x = 0
    yi = 0.015
    vi_y = 0
    a = 10
    b = 1

    Tc = (zc / g) ** (0.5)

    wp = array([[0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                 0.1, 0],
                [0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                 0.18, 0.18, 0.18, 0.18, 0.18, 0.18]])
    # wp = array([[0, 0.1, 0.1, 0.1, 0],
    #             [0.18, 0.18, 0.18, 0.18, 0.18 ]])
    # wp = array([[0, 0.30, 0.30, 0.30, 0],
    #             [0.20, 0.20, 0.2, 0.2, 0.2 ]])

    n = -1  # change this depending on T time
    N = len(wp.T)
    print(N)
    p = zeros([2, 100])  # px0 is 0 and py0 is 0 as it is right leg
    p_mod = zeros([2, N + 1])

    while True:
        if n == N - 1:
            break
        if n != -1:
            x_bar = (wp[0, n + 1]) / 2
            y_bar = (-1) ** (n + 1) * wp[1, n + 1] / 2

            px = p[0, n] + wp[0, n]
            py = p[1, n] - ((-1) ** (n + 1)) * wp[1, n]

            p[0, n + 1] = px
            p[1, n + 1] = py

            xd = px + x_bar
            yd = py + y_bar

            C = cosh(T_supp / Tc)
            S = sinh(T_supp / Tc)
            D = a * (C - 1) ** 2 + b * (S / Tc) ** 2

            vd_x = ((C + 1) / (Tc * S)) * x_bar
            vd_y = ((C - 1) / (Tc * S)) * y_bar

            px_mod = -(a * (C - 1) / D) * (xd - C * xi - Tc * S * vi_x) - (b * S / (Tc * D)) * (
                    vd_x - (S / Tc) * xi - C * vi_x)
            py_mod = -(a * (C - 1) / D) * (yd - C * yi - Tc * S * vi_y) - (b * S / (Tc * D)) * (
                    vd_y - (S / Tc) * yi - C * vi_y)
            p_mod[0, n + 1] = px_mod
            p_mod[1, n + 1] = py_mod
        else:
            px_mod = 0
            py_mod = 0
        C = cosh(t / Tc)
        S = sinh(t / Tc)
        D = a * (C - 1) ** 2 + b * (S / Tc) ** 2

        x = (xi - px_mod) * (C) + Tc * vi_x * S + px_mod
        y = (yi - py_mod) * (C) + Tc * vi_y * S + py_mod
        v_x = ((xi - px_mod) / Tc) * S + vi_x * C
        v_y = ((yi - py_mod) / Tc) * S + vi_y * C
        if n == -1:
            xsolve = copy(x)
            vxsolve = copy(v_x)
            ysolve = copy(y)
            vysolve = copy(v_y)
        else:
            xsolve = concatenate((xsolve, x[1:]))
            vxsolve = concatenate((vxsolve, v_x[1:]))
            ysolve = concatenate((ysolve, y[1:]))
            vysolve = concatenate((vysolve, v_y[1:]))

        n = n + 1
        xi = x[-1]
        vi_x = v_x[-1]
        yi = y[-1]
        vi_y = v_y[-1]
    return xsolve, vxsolve, ysolve, vysolve, p_mod

# T_supp = 0.6
# speed = 0.1
# zc = 0.6
# xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed,T_supp,zc)
# print(ysolve)
# t = linspace(0, 5* T_supp, len(ysolve))
# plt.plot(t, ysolve)
# plt.show()
