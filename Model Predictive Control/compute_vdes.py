import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

# Define constants
Crr = 0.01  # Rolling resistance coefficient
m = 500    # Mass of the vehicle (kg)
g = 9.8    # Acceleration due to gravity (m/s^2)
rho = 1.2 # Air density (kg/m^3)
Cd = 0.15    # Drag coefficient
Aref = 1  # Reference area (m^2)
eta = 0.9   # Efficiency

# Define the equation to solve
def equation(V):
    LHS = (1000 * 20000 / V) + 200000
    RHS = (20000 * (Crr * m * g + 0.5 * rho * V**2 * Cd * Aref) / eta) + 1000
    return LHS - RHS

# Initial guess for V
V_initial = 10  # Initial guess in m/s

# Solve the equation
V_solution = fsolve(equation, V_initial)            # fsolve is a function that solves the equation

# Display the solution
print(f"The solution for V is: {V_solution[0]:.2f} m/s")    

Vv = np.linspace(1, 30, 100)
eVv = [equation(V) for V in Vv]

plt.plot(Vv, eVv, label='Error (LHS- RHS)')
plt.xlabel('Velocity (m/s)')

# plot a line on x-axis, and annnoate a point where it intersecs the curve
plt.hlines(0, xmin=Vv[0], xmax=Vv[-1], color='k', linestyle='--', label='Error = 0')

# find where the valeu in np.array(eVv) is closest to 0
index = np.argmin(np.abs(eVv))
plt.scatter(Vv[index], 0, color='red', label=r'$V_{des}$ = %.2f m/s' %(Vv[index]))
plt.ylabel('Error')
plt.title('Error vs Velocity')
plt.legend()
plt.grid()

# SAVE THE FIGURE
plt.savefig('error_vs_velocity.png')