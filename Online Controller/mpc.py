import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import sys, os
import time

T_supp = 0.6
T_dbl = 0.08
speed = 0.1
zc = 0.6
g = 9.81
dt = 0.01
steps = 16
N = int(T_supp * steps / dt + 1)
N_Horizon = 180

top = [0.18 for x in range(int(T_supp / dt))]
bot = [0 for x in range(int(T_supp / dt))]

t_dsp = np.linspace(dt, T_dbl, int(int(T_dbl / dt)))

dsp_1 = [(0 - .18) * t / T_dbl + 0.18 for t in t_dsp]
dsp_2 = dsp_1.copy()
dsp_2.reverse()

p_ref_y = [0.09 for x in range(int(T_supp / dt))]
for i in range(int(steps)):
    p_ref_y = p_ref_y + top + dsp_1 + bot + dsp_2

p_ref_y = np.array(p_ref_y).reshape((len(p_ref_y), 1))
A = np.array([[1, dt, dt ** 2 / 2],
              [0, 1, dt],
              [0, 0, 1]])

B = np.array([[dt ** 3 / 6],
              [dt ** 2 / 2],
              [dt]])
C = np.array([[1, 0, -zc / g]])

model_type = 'discrete'  # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

_x = model.set_variable(var_type='_x', var_name='x', shape=(3, 1))
_y = model.set_variable(var_type='_x', var_name='y', shape=(1, 1))
_u = model.set_variable(var_type='_u', var_name='u', shape=(1, 1))
p_ref = model.set_variable(var_type='_tvp', var_name='p_ref')

x_next = A @ _x + B @ _u
y_next = C @ _x

model.set_rhs('x', x_next)
model.set_rhs('y', y_next)

# In[7]:


model.setup()

# In[8]:


mpc = do_mpc.controller.MPC(model)
setup_mpc = {
    'n_robust': 0,
    'n_horizon': N_Horizon,
    't_step': dt,
    'state_discretization': 'discrete',
    'store_full_solution': False,
    # Use MA27 linear solver in ipopt for faster calculations:
    'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
}

mpc.set_param(**setup_mpc)

# In[9]:


lterm = 1 * (model.x['y'] - model.tvp['p_ref']) ** 2
mterm = (model.x['y'] - p_ref_y[N_Horizon, 0]) ** 2
mpc.set_objective(mterm=mterm, lterm=lterm)
mpc.set_rterm(u=1e-8)  # input penalty

# In[10]:


mpc.bounds['lower', '_x', 'x'] = np.array([[-0.00], [-5], [-5]])
mpc.bounds['upper', '_x', 'x'] = np.array([[0.18], [5], [5]])

mpc.bounds['lower', '_x', 'y'] = np.array([[0]])
mpc.bounds['upper', '_x', 'y'] = -np.array([[-0.18]])

tvp_template = mpc.get_tvp_template()


def tvp_fun(t_now):
    index = int(t_now / dt)
    # for k in range(N_Horizon):
    #     tvp_template['_tvp', k, 'p_ref'] = p_ref_y[index + k, 0]

    tvp_template['_tvp', :N_Horizon + 1, 'p_ref'] = p_ref_y[index:index + N_Horizon + 1, :].ravel().tolist()

    return tvp_template


mpc.set_tvp_fun(tvp_fun)

mpc.setup()
simulator = do_mpc.simulator.Simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)
simulator.set_param(t_step=dt)

tvp_temp = simulator.get_tvp_template()


def tvp_fun(t_now):
    return tvp_temp


simulator.set_tvp_fun(tvp_fun)

simulator.setup()

x0 = np.array([[0.09], [0], [0], [0.09]])
mpc.x0 = x0
simulator.x0 = x0
estimator.x0 = x0
mpc.set_initial_guess()

time_now = time.time()
sys.stdout = open(os.devnull, "w")

for k in range(N_Horizon):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)

sys.stdout = sys.__stdout__
print(time.time() - time_now)
plt.plot(mpc.data['_time'], mpc.data['_x'][:, 0])
plt.plot(mpc.data['_time'], mpc.data['_x'][:, 3])
plt.plot(mpc.data['_time'], p_ref_y[:len(mpc.data['_time']), 0])

# In[18]:


plt.show()
# In[19]:
