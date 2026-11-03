import mujoco
import mujoco.viewer
import numpy as np
import time

franka_path = "robotics_mujoco\\franka_panda\\panda_grasp_scene.xml"

model = mujoco.MjModel.from_xml_path(franka_path)
data = mujoco.MjData(model)

print("Model loaded successfully")
print(f"nq (generalized coordinates): {model.nq}")
print(f"nv (degrees of freedom):     {model.nv}")
print(f"nu (actuators):              {model.nu}")
print(f"Number of bodies:            {model.nbody}")
print(f"Number of sites:             {model.nsite}")

print("\nBody names:")
for i in range(model.nbody):
    print(i, model.body(i).name)

print("\nJoint names:")
for i in range(model.njnt):
    print(i, model.joint(i).name)

print("\nActuator names:")
for i in range(model.nu):
    print(i, model.actuator(i).name)

# print("\nSite names:")
# for i in range(model.nsite):
#     print(i, model.site(i).name)

# Optional: open interactive viewer
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     while viewer.is_running():
#         mujoco.mj_step(model, data)
#         viewer.sync()

print("\nActuator ctrlrange:")
for i in range(model.nu):
    print(i, model.actuator(i).name, model.actuator_ctrlrange[i])

print("\nJoint ranges:")
for i in range(model.njnt):
    print(i, model.joint(i).name, model.jnt_range[i])

# Initialize the flag BEFORE the viewer
open_gripper = True          # start by commanding open

with mujoco.viewer.launch_passive(model, data) as viewer:

    last_toggle_time = time.time()

    while viewer.is_running():
        step_start = time.time()

        # Toggle only every 2.5 seconds
        if time.time() - last_toggle_time > 2.5:
            open_gripper = not open_gripper
            last_toggle_time = time.time()
            print("Gripper command:", "OPEN" if open_gripper else "CLOSE")

        # Set the actual control signal
        if open_gripper:
            data.ctrl[7] = 255        # <-- replace with the real OPEN value
        else:
            data.ctrl[7] = 0          # <-- replace with the real CLOSE value

        # Hold the arm roughly where it is (optional but recommended)
        # data.ctrl[0:7] = data.qpos[0:7]

        mujoco.mj_step(model, data)
        viewer.sync()

        # Real-time pacing (important)
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)