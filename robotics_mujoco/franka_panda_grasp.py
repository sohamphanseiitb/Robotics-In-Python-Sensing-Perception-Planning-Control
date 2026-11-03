import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

# made new class derived from gym.Env
class PandaGrasper(gym.Env):
    metadata = {"render_modes": ["human", "rgb-array"],"render_fps": 50}

    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        # load scene with franka panda and cube
        self.model = mujoco.MjModel.from_xml_path("robotics_mujoco\\franka_panda\\panda_grasp_scene.xml")
        self.data = mujoco.MjData(self.model)

        # ID each object in the scene
        self.cube_body_id = self.model.body("cube").id # comes from <body name="cube">

        # What is the hand?
        # `link7` is the last arm link. `hand` is the body that represents the gripper mounting/palm.
        # `left_finger` and `right_finger` are the 2 movable fingers
        # `hand` is the rigid part of the enf-effector that the fingers are attached to. 
        # Usig data.body("hand").xpos provides a convenient way for the end effector position
        self.hand_body_id = self.model.body("hand").id # comes from panda.xml <body name="hand" line 270

        # Gripper joints: used to grab cube
        self.finger1_joint = self.model.joint("finger_joint1")
        self.finger2_joint = self.model.joint("finger_joint2")

        self.finger1_qposadr = self.finger1_joint.qposadr[0]
        self.finger2_qposadr = self.finger2_joint.qposadr[0]

        # How do the body/joint IDs work?
        # Once mujoco loads the XML, every element, that has a `name` attribute is registered in internal name tables.
        
        # Every new action comprises:
        ## 7 arm joint values + 1 gripper command
        ## define action-space with low and high action values

        # where did the -2.9 to 2.9 come from?
        ## approximately joint limits of the Panda, (in radians)
        self.action_space = spaces.Box(low=np.array([-2.9]*7 + [0.0]),
                                       high=np.array([2.9]*7 + [1.0]),
                                       dtype=np.float64)

        # Observation:
        ## [qpos_arm[7], qvel_arm[7], gripper_opening(1), cube_pos(3), ee_pos(3)]
        ## positions and velocities of all 7 joints
        ## gripper state: open or close
        ## cube position, and end effector orientation
        obs_dim = 7 + 7 + 1 + 3 + 3
        self.observation_space = spaces.Box(low=-np.inf, 
                                            high= np.inf, shape=(obs_dim,), dtype=np.float64)
 
        # set initial state of the arm 
        # commonly used ready pose for the Panda, similar to one used in mnay papers and original Franka demos. 
        # nor arbit, comfortable configuration that keeps the arm in front of the robot with good manipulability and away from joint limits and singularities. 
        self.home_qpos = np.array([0, -0.785, 0, -2.356, 0., 1.571, 0.785])

        # set max number of steps per episode: before truncation/termination
        self.max_episode_steps = 300
        self.current_step = 0

    # Write a function to obtain the observation from the environment
    def _get_obs(self):

        # joint positions and velocities
        qpos_arm = self.data.qpos[0:7].copy()
        qvel_arm = self.data.qvel[0:7].copy() 

        # gripper opening value (take average of both the fingers : # ask)
        grip = 0.5 * (self.data.qpos[self.finger1_qposadr] + self.data.qpos[self.finger2_qposadr])

        # find position of cube and end effector
        cube_pos = self.data.body(self.cube_body_id).xpos.copy()
        ee_pos = self.data.body(self.hand_body_id).xpos.copy()

        return np.concatenate([qpos_arm, qvel_arm, [grip], cube_pos, ee_pos])

    # write function to reset the simulation
    """
    reset() is called at the beginning of every episode. 
    It puts the whole simulation into a valid, randomized starting state and return the first observation.

    What reset is really doing (summary)

        Seed the RNG (via super().reset)
        Clear the whole simulation state (mj_resetData)
        Put the arm into a known home configuration
        Place the cube at a random reachable location
        Call mj_forward so that all derived quantities (xpos, xmat, etc.) are up to date
        Return the first observation
    """
    def reset(self, seed=None, options=None):

        # call Gymnasium's base `reset`. Responsible for seeding the environment's internal random number generator.
        # After this call, `self.np_random` becomes available.
        super().reset(seed=seed) # ask

        # Reset the MuJoCo simulation
        mujoco.mj_resetData(self.model, self.data)

        # initialize arm to initial pose
        self.data.qpos[0:7] = self.home_qpos
        self.data.ctrl[0:7] = self.home_qpos
        self.data.ctrl[7] = 0.0 # start with the gripper open

        # Intialize random cube position for the arm # ask
        self.data.qpos[self.model.joint("cube_joint").qposadr[0]:self.model.joint("cube_joint").qposadr[0]+3] = np.array([self.np_random.uniform(0.4, 0.6),
                                                                                                                          self.np_random.uniform(-0.2, 0.2),
                                                                                                                          0.05])
        # ask where did th self.np_random came from

        # Orientation of the free joint can stay identity for now.

        """
        A body that has a <freejoint> does not store its position in body.pos permanently.
        
        body.pos in the model is only the initial offset. The actual pose of a free body lives in data.qpos at the addresses belonging to that free joint (7 numbers: 3 position + 4 quaternion).
        
        That is why we write:
        Pythonqposadr = self.model.joint("cube_joint").qposadr[0]

        self.data.qpos[qposadr : qposadr+3] = [x, y, z]
        Using self.model.body("cube").pos would only change the model definition, not the current simulation state.

        A free joint has 7 coordinates in qpos:

            3 for position
            4 for orientation (quaternion)

        If we do not touch the quaternion part, it stays at the default (1, 0, 0, 0) which means “no rotation”. That is what “identity” means. For the first version we accept that the cube always starts axis-aligned.
        
        """

        # step ahead
        mujoco.mj_forward(self.model, self.data)

        self.current_step = 0
        obs = self._get_obs()
        info = {}
        return obs, info

    # write a function to drive the observe-act-observe outer loop
    def step(self, action):

        self.current_step += 1

        # split action
        joint_targets = action[:7]
        grip_command = action[7]   # we are normalizing action between 0, 1 hence [0, 1] expected here

        # apply joint targets
        self.data.ctrl[0:7] = joint_targets
        self.data.ctrl[7] = grip_command * 255.0 # expand from [0, 1] to true value

        # step forward physics: ask what's the difference between mj_forward and mj_step
        # basically ask what is reset function doing in general

        """
        Diff between mj_forward and mj_step

        mj_forward — computes all kinematic and dynamic quantities (positions, velocities, accelerations, sensor values, etc.) without advancing time. 
        It is a “update everything that can be computed from the current state” call.
        
        mj_step — actually advances the simulation by one timestep (integrates the equations of motion, resolves contacts, applies actuators, etc.).

        In reset we only need the derived quantities to be consistent, so mj_forward is enough (and faster). 
        In step we want the world to move forward in time, so we use mj_step.
        """
        mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()

        # Reward Shaping

        '''
        MuJoCo (and almost all robotics) uses a right-handed coordinate system:

        +x forward (in front of the robot)
        +y left
        +z up

        So cube_pos[2] is the height of the cube above the ground. That is why we give a bonus when cube_pos[2] > 0.15.
        Current reward is only a dense shaping reward. We have not yet defined a proper success condition or a big terminal bonus. 
        That is intentional for v1 — first make the environment run, then improve the reward.
        '''
        ee_pos = self.data.body(self.hand_body_id).xpos
        cube_pos = self.data.body(self.cube_body_id).xpos
        dist = np.linalg.norm(ee_pos - cube_pos)

        reward = -dist        # larger the distance, worse the reward.

        # add a bonus when gripper gets close enough:
        if dist < 0.08:
            reward += 0.5 * grip_command

        # reward if the robot was able to lift
        if cube_pos[2] > 0.15:
            reward += 2.0 # ask how does it know cube_pos[2] = y position 


        """
        Why is terminated always False?
        Left out proper termination logic on purpose to focus on the mechanics.
        Typical things that should set terminated = True:

        Success: cube is grasped and lifted above a height for N consecutive steps
        Failure: cube falls off the table (cube_pos[2] < 0.01)
        Failure: robot self-collides badly or joint limits are violated hard
        Timeout is usually put in truncated (the max_episode_steps check), not terminated
        """
        terminated = False   # ask why it is always set to false? is there no case where it's absolute no-no? If roboto does this, episode waste.
        truncated = self.current_step >= self.max_episode_steps
        info = {"distance": dist, "cube_height": cube_pos[2]}

        return obs, reward, terminated, truncated, info

    def render(self):
        pass

    def close(self):
        pass

# Class Ends: write a test script

env = PandaGrasper()
obs, info = env.reset()
print("Obs shape:", obs.shape)
n_steps = 100

for _ in range(n_steps):
    action = env.action_space.sample()
    obs, reward, term, trunc, info = env.step(action)
    #print(f"Action: {action}")
    print(f"Gripper position: {obs[14]}")
    #print(f"reward: {reward:.3f}, dist: {info['distance']:.3f}")
    if term or trunc:
        break

env.close()