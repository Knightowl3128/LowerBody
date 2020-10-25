import numpy as np
import scipy.io as sio
from scipy.interpolate import CubicSpline


class Walking():

    def __init__(self, robot, T_ssp, T_dsp, dt):
        self.robot = robot
        self.T_ssp = T_ssp
        self.T_dsp = T_dsp
        self.dt = dt
        self.timer = 0  # the timer to switch between states

        self.state = 'Double Support'
        self.swing_leg = 'Left'
        self.foot_rise = 0.04
        self.heightList = None
        self.step_size = 0.1
        self.step_multiplier = 1
        self.first_step = True

        self.foot_pos_r = [0, -0.09, 0]
        self.foot_pos_l = [0, 0.09, 0]

        self.p_ref_x = sio.loadmat(r'D:\Matlab Projects\MPC\New MPC\p_ref_x.mat', struct_as_record=False)['p_ref_x']
        self.p_ref_y = sio.loadmat(r'D:\Matlab Projects\MPC\New MPC\p_ref_y.mat', struct_as_record=False)['p_ref_y']

    def sim(self, time):
        if self.state == 'Single Support':
            return self.single_support(time)
        if self.state == 'Double Support':
            return self.double_support(time, self.first_step)

    def start_single_support(self):
        t = np.linspace(0, self.T_ssp, int(self.T_ssp / self.dt))
        footHeightTraj = CubicSpline([0, self.T_ssp / 2, self.T_ssp], [0, self.foot_rise, 0],
                                     bc_type=(((1, 0)), (1, 0)))

        self.heightList = footHeightTraj(t)
        self.timer = 0

        if self.swing_leg == 'Left':
            step_start_x = self.foot_pos_l[0]
            foot_1d_spline = CubicSpline([0, self.T_ssp],
                                         [step_start_x, step_start_x + self.step_multiplier * self.step_size],
                                         bc_type=(((1, 0)), (1, 0)))
            self.foot_1d = foot_1d_spline(t)
            # self.foot_1d = np.linspace(step_start_x,step_start_x + self.step_multiplier*self.step_size,(int(self.T_ssp/self.dt)))
        elif self.swing_leg == 'Right':
            step_start_x = self.foot_pos_r[0]
            foot_1d_spline = CubicSpline([0, self.T_ssp],
                                         [step_start_x, step_start_x + self.step_multiplier * self.step_size],
                                         bc_type=(((1, 0)), (1, 0)))
            self.foot_1d = foot_1d_spline(t)
            # self.foot_1d = np.linspace(step_start_x, step_start_x + self.step_multiplier*self.step_size, (int(self.T_ssp / self.dt)))
        else:
            raise Exception('Invalid Swing Leg:', self.swing_leg)

        self.step_multiplier = 2

    def single_support(self, time):

        if self.T_ssp - self.timer >= 1e-8:  # temp. fix for timing error
            if self.swing_leg == 'Left':
                index = int(round(self.timer / self.dt, 4))  # Fixing up some kind of rounding error TODO
                self.foot_pos_l = [self.foot_1d[index], 0.09, self.heightList[index]]
                self.timer += self.dt
                return self.foot_pos_l, self.foot_pos_r
            elif self.swing_leg == 'Right':
                index = int(round(self.timer / self.dt, 4))
                self.foot_pos_r = [self.foot_1d[index], -0.09, self.heightList[index]]
                self.timer += self.dt
                return self.foot_pos_l, self.foot_pos_r
            else:
                raise Exception('Invalid Swing Leg:', self.swing_leg)

        if self.swing_leg == 'Left':
            self.swing_leg = 'Right'
        elif self.swing_leg == 'Right':
            self.swing_leg = 'Left'
        print('Single Support Ended at time:', time)
        self.timer = 0
        self.state = 'Double Support'
        return self.double_support(time)

    def double_support(self, time, first_step=False):
        if not first_step:
            T_dsp = self.T_dsp
        else:
            T_dsp = self.T_ssp  # The initial phase will have a double support  duration of T_ssp
        if T_dsp - self.timer >= 1e-8:
            self.timer += self.dt
            return self.foot_pos_l, self.foot_pos_r
        self.timer = 0
        self.state = 'Single Support'
        self.first_step = False
        self.start_single_support()
        print('Double Support Ended at time:', time)
        return self.single_support(time)


if __name__ == '__main__':
    T_supp = 0.6
    T_dbl = 0.08
    dt = 0.01
    foo = Walking(1, T_supp, T_dbl, dt)
    print(foo.p_ref_y)
    foo.setup_mpc()

    foo.start_single_support()
