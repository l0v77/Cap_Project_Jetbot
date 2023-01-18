import matplotlib.pyplot as plt
import numpy as np
import pyomo.environ as pyo
from pyomo.opt import SolverStatus, TerminationCondition

# Define Any Constants and Creat Optimal Path for Testing

t = np.linspace(0,5,300)
x_path = np.sin(t)
y_path = t

theta0 = np.pi/4 # this is defined by me now but may come from the top-view camera later

theta_path = [theta0]

for i in range(np.size(t) - 2):
  delta_y = y_path[i + 2] - y_path[i + 1]
  delta_x = x_path[i + 2] - x_path[i + 1]
  theta_path.append(np.arctan(delta_y/delta_x))

# Terminal State
theta_f = np.pi/4

theta_path.append(theta_f)

fig = plt.figure
plt.plot(x_path, y_path, 'k')

Ts = 0.1       # 10 samples per second
N = 5          # length of horizon
w_max = 10     # maximum anular speed [rad/s] 
a_max = 1      # maximum acceleration [rad/s^2]

# ICs:
x0 = x_path[0]   # initial state from the top-view camera
y0 = y_path[0]   # initial state from the top-view camera
omega_R0 = 0     # starts from stationary
omega_L0 = 0     # starts from stationary

# maximum number of iterations (Number of iterations cannot exceed number of optimal reference points)
max_iteration = np.size(x_path) - 1           # minums 2 because np.size gives num of elements in array and 
# number of states & inputs
nx = 3
nu = 2

# vehicle parameter
r_R = 0.03  # radius of right wheel [m]
r_L = 0.03  # radius of left wheel [m]
L   = 0.1   # shaft length [m]

# Finite constrained optimiation problem

def solve_cftoc(N, Ts, nx, nu, r_R, r_L, L, w_max, a_max, x0, xf):

  model = pyo.ConcreteModel()
  model.tidx = pyo.Set(initialize=range(0, N+1)) # length of finite optimization problem
  model.xidx = pyo.Set(initialize=range(0, nx))
  model.uidx = pyo.Set(initialize=range(0, nu))

  # create state and input variables trajectory:
  model.x = pyo.Var(model.xidx, model.tidx)
  model.u = pyo.Var(model.uidx, model.tidx)

  # cost Function:
  model.cost = pyo.Objective(expr = sum((model.x[0, t]-xf[0])**2 + (model.x[1, t]-xf[1])**2 + (model.x[2, t]-xf[2])**2 for t in model.tidx if t <= N - 1), sense=pyo.minimize)

  # initial conditions
  model.constarint1 = pyo.Constraint(model.xidx, rule=lambda model, i: model.x[i, 0] == x0[i])

  # modeling of the problem (eurler discretization of states)
  model.constarint2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.x[0, t+1] == model.x[0, t] + Ts*(r_R/2*pyo.cos(model.x[2, t])*model.u[0, t] + r_L/2*pyo.cos(model.x[2, t])*model.u[1, t])
                                    if t <= N - 1 else pyo.Constraint.Skip)

  model.constarint3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.x[1, t+1] == model.x[1, t] + Ts*(r_R/2*pyo.sin(model.x[2, t])*model.u[0, t] + r_L/2*pyo.sin(model.x[2, t])*model.u[1, t])
                                    if t <= N - 1 else pyo.Constraint.Skip)

  model.constarint4 = pyo.Constraint(model.tidx, rule=lambda model, t: model.x[2, t+1] == model.x[2, t] + Ts*(r_R/L*model.u[0, t] - r_L/L*model.u[1, t])
                                    if t <= N - 1 else pyo.Constraint.Skip)

  # input constraints (abs(angular speed) less and equal to 5 rad/s)
  model.constarint5 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] <= w_max
                                    if t <= N - 1 else pyo.Constraint.Skip)

  model.constarint6 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] >= -w_max
                                    if t <= N - 1 else pyo.Constraint.Skip)

  model.constarint7 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] <= w_max 
                                    if t <= N - 1 else pyo.Constraint.Skip)

  model.constarint8 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] >= -w_max 
                                    if t <= N - 1 else pyo.Constraint.Skip)

  # model.constarint9 = pyo.Constraint(model.uidx, rule=lambda model, i: model.u[i, 0] == u0[i])

  # desired states 
  # model.constarint10 = pyo.Constraint(model.xidx, rule=lambda model, i: model.x[i, N] == xf[i])

  # acceleration constraints
  # model.constarint11 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1] - model.u[0, t] <= a_max
  #                                   if t <= N - 1 else pyo.Constraint.Skip)

  # model.constarint12 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1] - model.u[0, t] >= -a_max
  #                                   if t <= N - 1 else pyo.Constraint.Skip)

  # model.constarint13 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1] - model.u[1, t] <= a_max
  #                                   if t <= N - 1 else pyo.Constraint.Skip)

  # model.constarint14 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1] - model.u[1, t] >= -a_max 
  #                                   if t <= N - 1 else pyo.Constraint.Skip)

  # call non-linear solver:
  results = pyo.SolverFactory('ipopt').solve(model)

  xOpt = np.asarray([[model.x[i,t]() for i in model.xidx] for t in model.tidx]).T
  uOpt = np.asarray([model.u[:,t]() for t in model.tidx]).T
  JOpt = model.cost()
  
  if str(results.solver.termination_condition) == "optimal":
      feas = True
  else:
      feas = False

  return [model, feas, xOpt, uOpt, JOpt]

# Run MPC and plot the states

def MPC_Sim(x0, y0, theta0, omega_R0, omega_L0, N, Ts, nx, nu, r_R, r_L, L, w_max, a_max, max_iteration):

  x_Opt = [x0]
  y_Opt = [y0]
  theta_Opt = [theta0]
  omega_R_Opt = [omega_R0]
  omega_L_Opt = [omega_L0]
  M = 0

  while True:

    x_init = [x_Opt[M], y_Opt[M], theta_Opt[M]]
    
    xf = [x_path[M+1], y_path[M+1], theta_path[M+1]]  # Every Ts, the reference states become the next states in the optimal sequence

    [model, feas, x, u, J] = solve_cftoc(N, Ts, nx, nu, r_R, r_L, L, w_max, a_max, x_init, xf)
    
    if not feas:
      print('Feasibility =', feas)
      break

    x_Opt.append(x[0, 1])
    y_Opt.append(x[1, 1])
    theta_Opt.append(x[2, 1])

    omega_R_Opt.append(u[0, 0])
    omega_L_Opt.append(u[1, 0])
    
    M += 1

    if M == max_iteration:
      break
    
  return [x_Opt, y_Opt, theta_Opt, omega_R_Opt, omega_L_Opt, M]

[x_Opt, y_Opt, theta_Opt, omega_R_Opt, omega_L_Opt, M] = MPC_Sim(x0, y0, theta0, omega_R0, omega_L0, N, Ts, nx, nu, r_R, r_L, L, w_max, a_max, max_iteration)

"""# State plots"""

tGrid = np.linspace(0, Ts*M, M+1)
# States vs time
fig = plt.figure(figsize=(16, 6))
xy = fig.add_subplot(1, 2, 1)
xy.plot(x_Opt, y_Opt, 'k')
xy.plot(x_path, y_path, '--k')
xy.set_xlabel('$x~(m)$')
xy.set_ylabel('$y~(m)$')
xy.title.set_text('Path of Vehicle (X-Y)')
xy.legend(['Real Path', 'Optimal Path'])

theta_plot = fig.add_subplot(1, 2, 2)
theta_plot.plot(tGrid, theta_Opt, 'k')
theta_plot.plot(tGrid, theta_path, '--k')
theta_plot.set_ylabel('$\phi~(rad)$')
theta_plot.set_xlabel('$t~(sec)$')
theta_plot.title.set_text('Global Heading Angle')
theta_plot.ticklabel_format(useOffset=False, style='plain')
theta_plot.legend(['Real Orientation', 'Optimal Orientation'])

fig.tight_layout() 

# input vs time
fig = plt.figure(figsize=(16, 6))
aa = fig.add_subplot(1, 2, 1)
aa.plot(tGrid, omega_R_Opt, 'k')
aa.set_ylabel('$\omega_R~(rad/s)$')
aa.set_xlabel('$t~(sec)$')
aa.title.set_text('Angular Speed of Right Wheel')
aa.ticklabel_format(useOffset=False, style='plain')

ab = fig.add_subplot(1, 2, 2)
ab.plot(tGrid, omega_L_Opt, 'k')
ab.set_ylabel('$\omega_L~(rad/s)$')
ab.set_xlabel('$t~(sec)$')
ab.title.set_text('Angular Speed of Left Wheel')
ab.ticklabel_format(useOffset=False, style='plain')

fig.tight_layout()