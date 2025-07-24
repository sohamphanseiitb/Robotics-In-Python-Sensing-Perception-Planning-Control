import unittest
import numpy as np
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from class_signatures import Sensor_Fusion, Sensor
from tests.test_cases_sensor import *

class FusionTest():

    def __init__(self):
        pass

class TestFusion(unittest.TestCase):
    pass


