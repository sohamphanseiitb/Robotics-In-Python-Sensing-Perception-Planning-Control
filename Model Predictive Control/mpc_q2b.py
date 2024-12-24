import numpy as np
from scipy.optimize import minimize
from tqdm import tqdm
import matplotlib.pyplot as plt
import seaborn as sns
import pickle

# System parameters
eta = 0.9                   # Efficiency of the motor
C_rr = 0.01                 # Rolling resistance coefficient
m = 500                     # Mass of the vehicle (kg)
g = 9.8                     # Gravitational acceleration (m/s^2)
rho = 1.2                   # Air density (kg/m^3)
C_d = 0.15                  # Drag coefficient
A_ref = 1                   # Reference area (m^2)
dx = 10            # Distance step (m)
dxc = 2                 # Distance step (m) for simulation of continuous system
xfinal = 20000              # in metres
xpreview = 500              # in metres
eps = 1e-10                  # small value to avoid division by zero

# Initial conditions
v0 = 15                     # in m/s
E_bat0 = 200000             # in J
x0 = 0                      # in m
t0 = 0                      # s

# MPC parameters
Np = 50                              # Prediction horizon
Nc = 50                              # Control horizon
sim_steps = int(xfinal/dx)           # Total simulation steps (20 km / 10 m) = 2000 steps

# Constraints
v_min, v_max = 0 , 30                # m/s
E_bat_min, E_bat_max = 1000, 200000  # J

# Solar power function
def solar_power(x):
    if x>=0 and x <= 5000:  # 0-5 km
        return 1000
    elif x>5000 and x <= 10000: # 5-10 km
        return 800
    elif x>10000 and x <= 15000: # 10-15 km
        return 1200
    else: # 15-20 km
        return 1000
    
# plot the solar power function using beautiful seaborn style
sns.set_context("talk")
sns.set_style("darkgrid")
sns.set_palette("husl")

x = np.linspace(0, xfinal, 1000)
y = [solar_power(i) for i in x]
plt.plot(x,y)
plt.xlabel('Distance (m)', fontsize=10)
plt.xticks(fontsize=10)
plt.yticks(fontsize=10)
plt.ylabel('Solar Power (W)', fontsize=10)
plt.title('Solar Power (in W) vs Distance (in km)', fontsize=10)
plt.grid(1)

# save figure
plt.savefig("solar_power.png")

# System dynamics
def system_dynamics(X, a, P_sun):
    E_bat, v = X                    # State variables
    dE_bat = P_sun - (v/eta) * (m*a + C_rr*m*g + 0.5*rho*v**2*C_d*A_ref)
    dv = a
    return np.array([dE_bat, dv])   # Derivative of the state

# RK4 method for state update - option 1
"""def rk4_update(X, a, dx):
    E_bat, v, t, x = X
    P_sun = solar_power(x)
    
    k1 = system_dynamics([E_bat, v], a, P_sun)
    k2 = system_dynamics([E_bat + dx/(2*v)*k1[0], v + dx/(2*v)*k1[1]], a, P_sun)
    k3 = system_dynamics([E_bat + dx/(2*v)*k2[0], v + dx/(2*v)*k2[1]], a, P_sun)
    k4 = system_dynamics([E_bat + dx/v*k3[0], v + dx/v*k3[1]], a, P_sun)
    
    E_bat_next = E_bat + (dx/v) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) / 6
    v_next = v + (dx/v) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) / 6
    t_next = t + dx/v
    x_next = x + dx
    
    return np.array([E_bat_next, v_next, t_next, x_next])"""

# RK4 method for state update - option 2 - more refined
def rk4_update(X, a, dx): # option 2

    E_bat, v, t, x = X
    P_sun = solar_power(x)
    
    # Update velocity using the equation of motion: v^2 = v_0^2 + 2aΔx
    v_next = v + a*dx/v #np.sqrt(v**2 + 2*a*dx + eps)
    
    # Calculate average velocity over the step
    v_avg = (v + v_next) / 2
    
    # Update battery energy using RK4
    k1 = P_sun - (v/eta) * (m*a + C_rr*m*g + 0.5*rho*v**2*C_d*A_ref)
    k2 = P_sun - (v_avg/eta) * (m*a + C_rr*m*g + 0.5*rho*v_avg**2*C_d*A_ref)
    k3 = P_sun - (v_avg/eta) * (m*a + C_rr*m*g + 0.5*rho*v_avg**2*C_d*A_ref)
    k4 = P_sun - (v_next/eta) * (m*a + C_rr*m*g + 0.5*rho*v_next**2*C_d*A_ref)
    
    E_bat_next = E_bat + (dx/v_avg) * (k1 + 2*k2 + 2*k3 + k4) / 6
    
    # Update time
    t_next = t + 2*dx / (v + v_next)
    
    # Update position
    x_next = x + dx
    
    return np.array([E_bat_next, v_next, t_next, x_next])

# Cost function for MPC
def cost_function(a, X):
    E_bat, v, t, x = X
    t_start = t
    J = 0
    for i in range(Np):
        X = rk4_update([E_bat, v, t, x], a[i], dx)
        E_bat, v, t, x = X
        J += 10 * a[i]**2

        # also penalize E_bat values less than 50% of the battery capacity
        #J += 0.1 * (E_bat - 0.5*E_bat_max)**2

    J += (t - t_start)
    return J

# Nonlinear constraints for optimization
def nlcon(a, X):
    E_bat, v, t, x = X
    c = []
    for i in range(Np):
        X = rk4_update([E_bat, v, t, x], a[i], dx)
        E_bat, v, t, x = X
        c.extend([v_min - v, v - v_max, E_bat_min - E_bat, E_bat - E_bat_max])
    return np.array(c)

# Main simulation loop
x_array = [x0]
v_array = [v0]
E_bat_array = [E_bat0]
t_array = [t0]
a_array = []

for k in tqdm(range(sim_steps - 1)):

    X = np.array([E_bat_array[k], v_array[k], t_array[k], x_array[k]])
    
    # Solve MPC optimization problem
    a_init = np.zeros(Np)
    bounds = [(-10, 10)] * Np
    cons = {'type': 'ineq', 'fun': lambda a: -nlcon(a, X)}
    result = minimize(lambda a: cost_function(a, X), a_init, method='SLSQP', bounds=bounds, constraints=cons)
    
    a = result.x[0]
    a_array.append(a)
    
    # Update state using RK4 method
    # simulate for a smaller time step dxc, hence keep a constant for the same

    ac = np.ones(int(dx/dxc)) * a
    for jj in range(int(dx/dxc)):
        X_next = rk4_update(X, ac[jj], dxc)
        X = X_next
    
    E_bat_array.append(X_next[0])
    v_array.append(X_next[1])
    t_array.append(X_next[2])
    x_array.append(X_next[3])
    
    # Break if we've reached the end of the race
    if x_array[-1] >= 20000:
        break

# Plot results
fig, axs = plt.subplots(3, 1, figsize=(12, 8))

# make beautiful plots using seaborn style
sns.set_context("talk")
sns.set_style("darkgrid")
sns.set_palette("husl")

axs[0].plot(np.array(x_array[:-1]), v_array[:-1])
axs[0].set_xlabel('Position in metres')
axs[0].set_ylabel('Velocity (m/s)')
axs[0].set_title('Velocity Profile')

axs[1].plot(np.array(x_array[:-1]), np.array(E_bat_array[:-1])/1000)
axs[1].set_xlabel('Position in metres')
axs[1].set_ylabel('Battery State of Charge (kJ)')
axs[1].set_title('Battery State of Charge')

axs[2].plot(np.array(x_array[:-1]), a_array)
axs[2].set_xlabel('Position in metres')
axs[2].set_ylabel('Acceleration (m/s^2)')
axs[2].set_title('Acceleration Profile')

print(f"Race completion time: {t_array[-1]:.2f} seconds")
plt.tight_layout()

# save figure
plt.savefig("mpc_results_2b.png")

# save the variables with pickle
with open('mpc_results_2b.pkl', 'wb') as f:
    pickle.dump([x_array, v_array, E_bat_array, a_array, t_array], f)