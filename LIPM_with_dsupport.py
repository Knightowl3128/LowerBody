from numpy import *
import matplotlib.pyplot as plt


def LIPM(speed, T_supp, T_dbl, zc, size):
    # zc = .6
    g = 9.810
    # initial condition
    xi = 0
    vi_x = 0
    yi = 0.015
    vi_y = 0
    a = 10
    b = 1

    Tc = (zc / g) ** (0.5)

    # wp = array([[0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0],
    #             [0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18]])

    first = .09 * ones((1, size))
    second = 0.18 * ones((1, size))
    temp = concatenate((first, second), axis=0)
    wp = array([[0],
                [0.18]])
    wp = concatenate((wp, temp), axis=1)

    # wp = array(
    #     [[0, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0],
    #      [0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
    #       0.18]])
    # wp = array([[0, 0.09, 0.09, 0.09, 0],
    #             [0.18, 0.18, 0.18, 0.18, 0.18, ]])

    n = -1  # change this depending on T time
    N = len(wp.T)
    p = zeros([2, size + 1])  # px0 is 0 and py0 is 0 as it is right leg
    p_mod = zeros([2, N + 1])

    while True:
        if n == N - 1:
            break
        t = linspace(0, T_supp, T_supp / speed + 1)
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
            p_mod[1, n + 1] = py_mod - 0.09
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
        a_x = ((xi - px_mod) / (Tc ** 2)) * C + vi_x * S / Tc
        a_y = ((yi - py_mod) / (Tc ** 2)) * C + vi_y * S / Tc

        A = array([[2, 3 * T_dbl, 3 * (T_dbl ** 2)],
                   [2 * T_dbl, 3 * (T_dbl ** 2), 4 * T_dbl ** 3],
                   [2, 0, 0]])
        coeff_x = array([[0], [0], [a_x[-1]]])
        coeff_y = array([[0], [0], [a_y[-1]]])
        X = linalg.solve(A, coeff_x)
        Y = linalg.solve(A, coeff_y)
        a0 = x[-1]
        a1 = v_x[-1]
        a2 = X[0, 0]
        a3 = X[1, 0]
        a4 = X[2, 0]
        t = linspace(0, T_dbl, T_dbl / speed + 1)
        dbl_x = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4
        dbl_vx = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3
        a0 = y[-1]
        a1 = v_y[-1]
        a2 = Y[0, 0]
        a3 = Y[1, 0]
        a4 = Y[2, 0]
        dbl_y = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4
        dbl_vy = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3

        if n != N - 2:
            x = concatenate((x, dbl_x[1:]))
            v_x = concatenate((v_x, dbl_vx[1:]))
            y = concatenate((y, dbl_y[1:]))
            v_y = concatenate((v_y, dbl_vy[1:]))
        if n == -1:
            xsolve = x
            vxsolve = v_x
            ysolve = y
            vysolve = v_y
        else:
            xsolve = concatenate((xsolve, x[1:]))
            vxsolve = concatenate((vxsolve, v_x[1:]))
            ysolve = concatenate((ysolve, y[1:]))
            vysolve = concatenate((vysolve, v_y[1:]))

        n = n + 1
        xi = xsolve[-1]
        vi_x = vxsolve[-1]
        yi = ysolve[-1]
        vi_y = vysolve[-1]

    return xsolve, vxsolve, ysolve, vysolve, p_mod
#
#
# T_supp = 0.5
# T_dbl = 0.1
# speed = 0.01
# zc = 0.6
# xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed, T_supp, T_dbl, zc)
#
# t = linspace(0, (4)*(T_supp+T_dbl), len(vxsolve))
# # print(p_mod)
# plt.plot(t, vysolve)
# plt.show()
