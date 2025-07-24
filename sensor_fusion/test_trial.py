import unittest
import numpy as np
from class_signatures import DynamicsModel2D

class TestDynamicsModel2D(unittest.TestCase):

    def test_simulate_motion(self):
        dt = 0.1
        timestamps = np.arange(0, 1, dt)
        acceleration_x = np.zeros_like(timestamps)
        acceleration_y = np.ones_like(timestamps)
        acceleration_z = np.zeros_like(timestamps)
        model = DynamicsModel2D(dt, timestamps, acceleration_x, acceleration_y, acceleration_z)
        model.simulate_motion()
        self.assertAlmostEqual(model.motion_y[-1], 0.5 * (1**2), places=5) # Expected y position after 1s

if __name__ == '__main__':
    unittest.main()