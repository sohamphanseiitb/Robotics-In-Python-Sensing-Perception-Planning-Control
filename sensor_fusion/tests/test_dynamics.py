import unittest
import numpy as np
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from class_signatures import DynamicsModel2D
from test_cases_dynamics import *

class DynamicsTest():
    def __init__(self, dt: float, tstart: float, t_end: float, initial_cords: np.array, accx: np.array, accy: np.array, accz: np.array, expected_results: np.array):
        self.dt = dt
        self.tstart = tstart
        self.t_end = t_end
        self.timestamps = np.arange(self.tstart, self.t_end, self.dt)
        self.initial_cords = initial_cords
        self.accx = accx
        self.accy = accy
        self.accz = accz
        self.expected = expected_results

class TestDynamicsModel2D(unittest.TestCase):

    def run_simulation_test(self, test_data):
        model = DynamicsModel2D(test_data.dt, test_data.timestamps, test_data.initial_cords, test_data.accx, test_data.accy, test_data.accz)
        model.simulate_motion()
        np.testing.assert_allclose(
            [model.motion_x[-1], model.motion_y[-1], model.motion_z[-1], model.velx[-1], model.vely[-1], model.velz[-1]],
            test_data.expected,
            atol=1e-5
        )

    def test_case_1(self):
        self.run_simulation_test(dyna_test1)

    def test_case_2(self):
        self.run_simulation_test(dyna_test2)

    def test_case_3(self):
        self.run_simulation_test(dyna_test3)

if __name__ == '__main__':
    unittest.main()