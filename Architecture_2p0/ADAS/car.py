"""
DOCSTRING:
This class implements a car model to be used for implementing ADAS systems. 

The state of the car can be described as the position [x, y], yaw angle psi, and velocity v

Dynamics: d state/dt = v cos(psi + beta), v sin(psi + beta), (v/Lr)sin beta, a

beta = arctan((Lr/(Lr + Lf)) * arctan (delta))
"""