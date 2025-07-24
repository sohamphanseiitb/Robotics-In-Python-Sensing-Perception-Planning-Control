import unittest
import numpy as np
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from class_signatures import Sensor_Fusion, Sensor
from tests.test_cases_sensor import *

class SensorTest():

    def __init__(self, sens_traj: dict, gcf_sens_R: np.array, gcf_sens_T: np.array, expected_results: list):
        self.sens_traj = sens_traj
        self.gcf_sens_R = gcf_sens_R
        self.gcf_sens_T = gcf_sens_T
        self.expected_results = expected_results

class TestSensor(unittest.TestCase):

    def run_test_case(self, test_case):

        np.testing.assert_allclose(test_case.sens_traj['x'][-1], test_case.expected_results[0][0], atol=5)
        np.testing.assert_allclose(test_case.sens_traj['y'][-1], test_case.expected_results[0][1], atol=5)
        np.testing.assert_allclose(test_case.sens_traj['z'][-1], test_case.expected_results[0][2], atol=5)
        np.testing.assert_allclose(test_case.sens_traj['theta_z'][100], test_case.expected_results[0][3], atol=5)
        np.testing.assert_allclose(test_case.gcf_sens_R, test_case.expected_results[1], atol=0.1)
        np.testing.assert_allclose(test_case.gcf_sens_T, test_case.expected_results[2], atol=0.1)

    
    def test_case1(self):
        self.run_test_case(sens_test1)

if __name__ == '__main__':
    unittest.main()