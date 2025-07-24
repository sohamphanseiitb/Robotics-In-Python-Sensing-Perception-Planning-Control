# Internship Assignment - Soham Phanse

1. The ```main.py``` contains the main implementation of the project. If you run the main file, then you will obtain 3 plots: comparison between each sensor measurement and ground truth and finally the fused data.
2. The ```debug.ipynb``` contains trial codes written by me for experimentation. 
3. The ```dynamics.py``` implements a serpentine trajectory generation for the lawnmower robot. 
4. The ```class_signatures.py``` implements the following classes:
    - LawnMowerField: parameters and methods for the field where the LawnMower moves
    - LawnMowerRobot: parameters for the bot, motion propagation for bot
    - Sensor: sensor simulation, noise models, co-ordinate frame transformation, timestamo interpolation
    - Animator: generates motion animations 
    - Plotter: generates comparison plots for the sensor co-ordinates and ground truth
    - DynamicsModel2D: customized trajectory generation
    - SensorFusion: Implements Sensor Fusion, and error metrics
5. Check the ```tests``` folder for the test automation codes
    - ```test_cases_dynamics.py``` and ```test_dynamics.py``` implements automated testing for dynamics models
    - ```test_cases_sensor.py``` and ```test_sensor.py``` implements automated testing for sensor model functionality
    - ```test_cases_fusion.py``` and ```test_fusion.py``` implements automated testing for sensor model functionality, but in the interest of time, and for a reasonable stopping point for this micro-assignment we keep it as future work. 
6. ```sensor_simulation.py``` files implements sensor models along with timestamp matching and interpolation between the ground truth sampling and the sensor sampling, incorporating noise models, as well as incorporating noise models
7. ```test_trial.py``` is a file where I wrote some codes for the automated testing framework. 