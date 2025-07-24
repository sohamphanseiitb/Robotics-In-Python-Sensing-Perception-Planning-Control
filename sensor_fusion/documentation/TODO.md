# To Do List

## For 3D

The dynamics model currently does not the capability to simulate 3D object dynamics. We will need to update it and include 3 Euler angles in $\alpha, \beta, \gamma$ instead of just the Euler angle in it. Other than that the transformations in the sensor trajectory generation are pretty generation and can be used. The system states everywhere will have to be updated with these new Euler angles.

## Implement Testing for fusion

The skeleton for testing the fusion is provided in ```test_cases_fusion.py```, and ```test_fusion.py```, as well as there counterparts in the sensor test files. However in the interest of time, and a reasonable ending point to the micro-project we decide stopping the work here. 