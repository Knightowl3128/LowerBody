import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg as sci
from scipy.optimize import fsolve

T_supp = 0.6
speed = 0.1
zc = 0.6
g = 9.81
dt = 0.01
steps = 6
N = int(T_supp * steps / dt + 1)
t = np.linspace(0, steps * T_supp, int(steps * T_supp / dt) + 1)

# Creating p_ref

top = [0.18 for x in range(int(T_supp / dt))]
bot = [0 for x in range(int(T_supp / dt))]

p_ref_y = [0.09 for x in range(int(T_supp / dt))]
for i in range(int(steps)):
    p_ref_y = p_ref_y + top + bot


def ricatti(x):
    x = x.reshape((3, 3))
    return (A.T @ x @ A + C.T @ Q @ C - A.T @ x @ B @ np.linalg.inv(R + B.T @ x @ B) @ B.T @ x @ A - x).ravel()


p_ref_y = np.array(p_ref_y).reshape((len(p_ref_y), 1))

# defining State Space Matrices
A = np.array([[1, dt, dt ** 2 / 2],
              [0, 1, dt],
              [0, 0, 1]])

B = np.array([[dt ** 3 / 6],
              [dt ** 2 / 2],
              [dt]])
C = np.array([[1, 0, -zc / g]])

Q = 1 * np.identity(1)
R = 1e-8 * np.identity(1)

ha = np.identity(3)
X = fsolve(ricatti, ha.ravel())
X = X.reshape((3, 3))
K = sci.inv(B.T @ X @ B + R) @ (B.T @ X @ A)
f = np.zeros((int(T_supp * steps / dt + 1), 1))

foo = sci.inv(B.T @ X @ B + R)
bar = (A - B @ K).T

for i in range(1, int(T_supp * steps / dt + 1) + 1):
    f_i = foo @ B.T @ np.linalg.matrix_power(bar, i - 1) @ C.T @ Q
    f[i - 1] = f_i
    # f.append(f_i)

X = np.zeros(((N, 3, 1)))
X[0, 0, 0] = 0.09
# print(X)
# print(K)
p_actual = np.zeros((1, N))
UHistory = np.zeros((1, N))
for index, val in enumerate(t):
    if index == N - 1:
        break
    u_k = -K @ X[index] + f.T @ p_ref_y[index + 1:index + N + 1, 0]
    X[index + 1] = A @ X[index] + B @ u_k

    p_actual[0, index] = C @ X[index]
    UHistory[0, index] = u_k
# print(X[:,0])
plt.plot(t, p_ref_y[0:360], label='plan zmp')
plt.plot(t[0:360], X[:, 0], label='CoM Pos')
# plt.plot(t[0:360],X[:,1],label = 'CoM Vel')
plt.plot(t[0:360], p_actual.T, label='actual zmp')
plt.plot(t[0:360], UHistory.T, label='actual zmp')
plt.legend()
plt.show()
