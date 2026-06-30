# Problem Statement

Till now, we have installed:
1. Pytorch with CUDA
2. MuJoCo
3. Stable Baselines 3
4. Gymnasium robosuite

## What are we trying to solve?
Make a robot arm insert a peg in a hole accurately. Simple but hard to do on robots. What's so hard about this? Lets say we define the position of the hole to be (x_h, y_h, z_h), and the position of the peg to be (x_p, y_p, z_p). Because of sensor uncertainties or so. We don't know the exact co-ordinates. And the robot arm is stiff, hence there's a chance that the peg will slam right on the lip of the hole. 

So whats the trick to make this work? Stop being stif. Make the arm compliant. Let it "give in" when it touches something, so contact forces gently guide the peg intpo alignment, the way you wiggle a key into a lock instead of aiming for it. Contact is messy and varies, hence a learned correction on top of a handtuned controller will just do the job. 

Classical control makes contact possible; the learned policy makes it robust. Peg-in-hole is starter pack for harder problems like electronics assembly, surgical instruments or so.

## What is robosuite and MuJoCo?
MuJoCo - Multi-Joint Dynamics with Contact  - is the thing that actually computes the physics. How joints move under torque, how bodies collide, how contact forces resolve. Robosuite is the python library which ships (and sits on top of MuJoCo):

1. Robot models: Franka Panda and so on
2. Tasks/Scenes: pre-built environments, like peg-insertion-style, peg, hole and the contact physics already set up.
3. Sensors: joint encoders, end-effector pose, force/torque sensor
4. Controllers: baseline controllers you can use as starters
5. API: env.reset(), env.step(action)

## What is the Franka Panda?
Its a real, commercial robot made by Agile Robots, based in Munich. Robosuite ships a detailed model of that robot. It has 7-DOF, and where 6-DOF is enough to place an end effector in any position and orientation - 7th DOF is spare, meaning you have redundancy, to avoid null spaces. If you remember from your rotations class, there's always a reachable space for any system.
- Specs: 2-finger parallel gripper at the wrist. The peg is held by the gripper (or rigidly attached at the end-effector) - the "hand" frame we'll command.

## Finale
A cylindrical hole (d_h = 10mm h_h = 10mm) sits at world frame co-ordinates (x_h, y_h, z_h), hole axis is vertical (aligned with world z). A Franka Panda must insert a peg (d_p = 9mm, h_p = 10mm) into it - giving radial clearance r = 0.5mm. A wrist camera reports the hole pose relative to the peg, corrupted by (a) zero-mean per-frame noise of +-2mm and (b) a fixeed per-episode offset of upto +8mm/axis that is unknown to the controller. **Success**: peg fully seated (inserted to depth = 10mm) within 10s, with the contact-force magnitude ||F|| = \sqrt(F_x**2 + F_y**2 + F_z**2) never exceeeding 50N at any moment. Any breach of force or time limit is failure. 

## Franka Panda's Observability
Here's the core problem is to setup the environment right: you need to simulate a noisy camera in order to demonstrate your controller and filter work. Let's look at what all sensors Franka Panda has:

1. robot0_joint_pos: (7,) (radians): the 7 joint angles
2. robot0_joint_vel: (7,) (radians/sec): joint angular velocities
3. robot0_eef_pos: (3,) (meters): end-effector (peg) position
4. robot0_eef_quat: (4,) (unitless): end-effector orientation, quaternion (x, y, z, w) order
5. robot0_eef_force: (3,) (N): contact force at the wrist
6. robot0_eef_torque: (3,) (N): contact torque
7. hole_pos, peg_pos, ...: (3,) (meters): task object ground truth

If we get and *use* the exact hole_pos, then that's pointless. Inputs like joint encoders, end-effector pose from forward kinematics, force/torque sensor. hole_pos and other readings cannot be used directly.

How do we construct the noisy camera?
1. True camera reading: z_cam = hole_pos - peg_pos
2. Add bias: upto +-8mm fixed per episode of peg insertion
3. Add zero mean noise $$\sigma = 2mm$$