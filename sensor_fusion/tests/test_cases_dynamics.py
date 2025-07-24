from test_dynamics import DynamicsTest
import numpy as np

# acceleration y constant at 1 m/s^2 for 20 seconds: 
## expected: y_pos = ut + 0.5*a*t^2 = 200.0 metres
## expected: vy = u + at = 0 + 1*20 = 20 m/s
dyna_test1 = DynamicsTest(
            dt=0.1,
            tstart=0,
            t_end=20,
            initial_cords=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            accx=np.zeros(200),
            accy=np.ones(200),
            accz=np.zeros(200),
            expected_results=np.array([0.0, 200.0, 0.0, 0.0, 20.0, 0.0]), # x, y, z, vx, vy, vz
        )

# acceleration x: constant at 1 m/s2 for the first 10 seconds, then -1 m/s2 for the next 10 seconds:
## x_pos(10) = 50, y_pos(10) = 0, z_pos(10) = 0, vx(10) = 10, vy(10) = 0, vz(10) = 0
## x_pos(20) = 100, y_pos(20) = 0, z_pos(20) = 0, vx(20) = 0, vy(20) = 0, vz(20) = 0
## expected: 100, 0, 0, 0, 0, 0
accxt2 = np.ones(200)
for i in range(100, 200): accxt2[i] = -1
dyna_test2 = DynamicsTest(
            dt=0.1,
            tstart=0,
            t_end=20,
            initial_cords=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            accx=accxt2,
            accy=np.zeros(200),
            accz=np.zeros(200),
            expected_results=np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )

# Follows:
"""
0 < t < 5: ax = 0, ay = 1, az = 0
5 < t < 10: ax = 1, ay = 0, az = 0
10 < t < 15: ax = 0, ay = -1, az = 0
15 < t < 20: ax = -1, ay = 0, az = 0
"""
from params import get_acceleration
timestampst3 = np.arange(0, 20, 0.1)
acceleration_x = np.array([get_acceleration(ti)[0] for ti in timestampst3])
acceleration_y = np.array([get_acceleration(ti)[1] for ti in timestampst3])
acceleration_z = np.array([get_acceleration(ti)[2] for ti in timestampst3])
dyna_test3 = DynamicsTest(
            dt=0.1,
            tstart=0,
            t_end=20,
            initial_cords=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            accx=acceleration_x,
            accy=acceleration_y,
            accz=acceleration_z,
            expected_results=np.array([49.0, 49.0, 0.0, 0.0, 0.0, 0.0]),
        )