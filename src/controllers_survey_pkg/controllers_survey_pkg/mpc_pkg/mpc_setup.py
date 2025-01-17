"""

"""

import numpy as np
# from casadi import *
from casadi import mtimes, vertcat, vertsplit
# from casadi.tools import *
import do_mpc
from controllers_survey_pkg.mpc_pkg.model import BicycleModel


class MPC(object):
    """docstring for ClassName"""

    def __init__(self, vehicle:BicycleModel, horizon=15, sample_time=0.02, wheelbase=0.256,
                 Q=np.diag([1e-1, 1e-8, 1e-8, 1e-8]), R=np.diag([1e-3, 5e-3]),
                 Qf=np.diag([0.0, 0.0, 0.0, 0.0]), Rd=np.diag([0.0, 0.0]),
                 vel_bound=(-5.0, 5.0), delta_bound=(-69.0, 69.0), acc_bound=(-1.0, 1.0),
                 max_iterations=20, tolerance=1e-6, suppress_ipopt_output=True):
        """Constructor for MPC"""
        self.model = vehicle.model

        self.horizon = horizon
        self.Q = Q
        self.R = R
        self.Qf = Qf
        self.Rd = Rd
        self.wheelbase = wheelbase

        self.Ts = sample_time
        self.reference_states = np.zeros((self.horizon + 1, self.Q.shape[0]))

        self.mpc = self.initialize_mpc(model=self.model, horizon=self.horizon,timestep=self.Ts, store_full_solution=True)

        nlpsol_opts = {
            'ipopt.max_iter': max_iterations,
            'record_time': True,
            'ipopt.acceptable_obj_change_tol': tolerance,
            # 'ipopt.linear_solver': 'MA27'
        }
        if suppress_ipopt_output:
            nlpsol_opts.update({"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0})

        self.mpc.set_param(nlpsol_opts=nlpsol_opts)

        # define the objective function and constraints
        self.objective_function_setup()
        self.constraints_setup(vel_bound=vel_bound, delta_bound=delta_bound, acc_bound=acc_bound, reset=False)

        # provide time-varing parameters: setpoints/references
        self.tvp_template = self.mpc.get_tvp_template()
        self.mpc.set_tvp_fun(self.tvp_fun)

        self.mpc.setup()

    def initialize_mpc(self, model, horizon, timestep=0.01, store_full_solution=False):
        """

        :param model:
        :param horizon:
        :param timestep:
        :param store_full_solution:
        :return:
        """
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': horizon,
            'n_robust': 0,  # Robust horizon for robust scenario-tree MPC,
            'open_loop': 0,
            't_step': timestep,
            'state_discretization': 'collocation',  # no other option at the moment
            'collocation_type': 'radau',  # no other option at the moment
            'collocation_deg': 2,
            'collocation_ni': 2,
            'store_full_solution': store_full_solution,  # re
            'store_solver_stats': ['success',
                                   # 'return_status',
                                   't_proc_callback_fun', 't_proc_nlp_f', 't_proc_nlp_g', 't_proc_nlp_grad',
                                   't_proc_nlp_grad_f', 't_proc_nlp_hess_l',
                                   't_proc_nlp_jac_g', 't_proc_S', 't_wall_callback_fun',  # t_proc_{S or total}
                                   't_wall_nlp_f', 't_wall_nlp_g', 't_wall_nlp_grad',
                                   't_wall_nlp_grad_f', 't_wall_nlp_hess_l', 't_wall_nlp_jac_g', 't_wall_total'],
            # Use MA27 linear solver in ipopt for faster calculations:
            # 'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}  # highly recommended (MA27), 'mumps'
        }
        mpc.set_param(**setup_mpc)
        return mpc
        
    def tvp_fun(self, t_now):
        """
        provides data into time-varying parameters.
        """
        # self.tvp_template['_tvp', 0] = [xk, yk, vel_k, psi_k]
        self.tvp_template['_tvp', :] = np.vsplit(self.reference_states, self.horizon + 1)
        # or self.tvp_template['_tvp', 0:(self.horizon + 1)] = casadi.vertsplit(self.reference_states)

        return self.tvp_template

    def objective_function_setup(self):
        # path following
        lterm = (self.Q[0, 0] * (self.model.x['pos_x'] - self.model.tvp['x_ref']) ** 2
                 + self.Q[1, 1] * (self.model.x['pos_y'] - self.model.tvp['y_ref']) ** 2
                 + self.Q[2, 2] * (self.model.x['vel'] - self.model.tvp['vel_ref']) ** 2
                 + self.Q[3, 3] * (self.model.aux['psi_diff']) ** 2
                 + self.R[0, 0] * (self.model.u['acc']) ** 2
                 + self.R[1, 1] * (self.model.u['delta']) ** 2
                 )  # stage/lagrange cost
        mterm = (self.Qf[0, 0] * (self.model.x['pos_x'] - self.model.tvp['x_ref']) ** 2
                 + self.Qf[1, 1] * (self.model.x['pos_y'] - self.model.tvp['y_ref']) ** 2
                 + self.Qf[2, 2] * (self.model.x['vel'] - self.model.tvp['vel_ref']) ** 2
                 # + self.Qf[3, 3] * (self.model.x['psi'] - self.model.tvp['psi_ref']) ** 2
                 )  # terminal/mayer cost
        
        self.mpc.set_objective(lterm=lterm, mterm=mterm)
        self.mpc.set_rterm(acc=self.Rd[0, 0], delta=self.Rd[1, 1])  # input penalty

    def constraints_setup(self, vel_bound=None, delta_bound=None, acc_bound=None, reset=False):

        # states constraints
        if delta_bound is None:
            delta_bound = [-69.0, 69.0]

        if vel_bound is None:
            vel_bound = [-10.0, 10.0]

        if acc_bound is None:
            acc_bound = [-1.0, 1.0]

        self.mpc.bounds['lower', '_x', 'vel'] = vel_bound[0]
        self.mpc.bounds['upper', '_x', 'vel'] = vel_bound[1]

        # input constraints
        self.mpc.bounds['lower', '_u', 'acc'] = acc_bound[0]
        self.mpc.bounds['upper', '_u', 'acc'] = acc_bound[1]
        self.mpc.bounds['lower', '_u', 'delta'] = np.radians(delta_bound[0])
        self.mpc.bounds['upper', '_u', 'delta'] = np.radians(delta_bound[1])

        if reset:
            self.mpc.setup()

    def get_control(self, x0):

        # solve optization problem
        u0 = self.mpc.make_step(x0)

        return u0

    def update(self, reference):
        # todo: to update reference used by tvp instead of calculating
        self.reference_states = reference
        pass

    def distance_update(self, states, s):
        vel, psi = states[2], states[3]

        # Compute velocity along path
        s_dot = vel * np.cos(self.mpc.data['_aux', 'psi_diff'][0])

        # Update distance travelled along reference path
        s += s_dot * self.Ts

        return s
    


def initialize_mpc_problem(horizon=15, sample_time=0.02,
                           Q=None, R=None, Qf=None, Rd=None, wheelbase=0.256,
                           delta_min=-23.0, delta_max=23.0, vel_min=-10.0, vel_max=10.0,
                           acc_min=-3.0, acc_max=3.0,
                           max_iterations=100, tolerance=1e-6, suppress_ipopt_output=True, model_type='continuous'):
    """
    Get configured do-mpc modules:
    """
    # model setup
    Vehicle = BicycleModel(wheelbase=wheelbase, width=0.192, sample_time=sample_time, model_type=model_type)

    Controller = MPC(Vehicle, horizon=horizon, sample_time=sample_time, Q=Q, R=R, Qf=Qf, Rd=Rd, wheelbase=wheelbase,
                     vel_bound=(vel_min, vel_max), delta_bound=(delta_min, delta_max), acc_bound=(acc_min, acc_max),
                     max_iterations=max_iterations, tolerance=tolerance, suppress_ipopt_output=suppress_ipopt_output)

    return Controller