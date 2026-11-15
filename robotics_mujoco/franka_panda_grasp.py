import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
import mujoco.viewer

# made new class derived from gym.Env
class PandaGrasper(gym.Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 50
        }

    def __init__(self, render_mode=None):

        super().__init__()
        self.render_mode = render_mode

        # load scene with franka panda and cube
        self.model = mujoco.MjModel.from_xml_path("robotics_mujoco\\franka_panda\\panda_grasp_scene.xml")
        self.data = mujoco.MjData(self.model)

        # ID each object in the scene
        self.cube_body_id = self.model.body("cube").id # comes from <body name="cube">

        '''
        # What is the hand?

        1. `link7` is the last arm link. `hand` is the body that represents the gripper mounting/palm.
        2. `left_finger` and `right_finger` are the 2 movable fingers
        3. `hand` is the rigid part of the enf-effector that the fingers are attached to. 
        4. Usig data.body("hand").xpos provides a convenient way for the end effector position

        '''
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
        ## [qpos_arm[7], qvel_arm[7], finger1_pos(1), finger2_pos(1), rel_pos(3), dist(1), 
        ##  left_tip_pos(3), right_tip_pos(3), left_contact(1), right_contact(1)]
        ## = 7 + 7 + 1 + 1 + 3 + 1 + 3 + 3 + 1 + 1 = 28
        obs_dim = 28
        self.observation_space = spaces.Box(low=-np.inf, 
                                            high=np.inf, shape=(obs_dim,), dtype=np.float64)
 
        # set initial state of the arm 
        # commonly used ready pose for the Panda, similar to one used in mnay papers and original Franka demos. 
        # nor arbit, comfortable configuration that keeps the arm in front of the robot with good manipulability and away from joint limits and singularities. 
        self.home_qpos = np.array([0, -0.785, 0, -2.356, 0., 1.571, 0.785])

        # set max number of steps per episode: before truncation/termination
        self.max_episode_steps = 300
        self.current_step = 0

        # multiple simulation substeps
        self.n_substeps = 5

        # rendering
        self.viewer = None          # we will create it lazily

        self.success_count = 0

    # Write a function to obtain the observation from the environment
    def _get_obs(self):

        # joint positions and velocities
        qpos_arm = self.data.qpos[0:7].copy()
        qvel_arm = self.data.qvel[0:7].copy() 

        # gripper opening value (take average of both the fingers : # ask)
        # grip = 0.5 * (self.data.qpos[self.finger1_qposadr] + self.data.qpos[self.finger2_qposadr])

        # Individual finger positions (not average!)
        finger1_pos = self.data.qpos[self.finger1_qposadr]
        finger2_pos = self.data.qpos[self.finger2_qposadr]

        # find position of cube and end effector
        cube_pos = self.data.body(self.cube_body_id).xpos.copy()
        ee_pos = self.data.body(self.hand_body_id).xpos.copy()

        relative_pos = cube_pos - ee_pos
        distance = np.linalg.norm(relative_pos)

        # add finger tip positions for contact detection
        left_tip_pos = self.data.site("left_fingertip").xpos.copy()
        right_tip_pos = self.data.site("right_fingertip").xpos.copy()
        left_contact = self._check_contact("left_finger", "cube")
        right_contact = self._check_contact("right_finger", "cube")
        
        # return np.concatenate([qpos_arm, qvel_arm, [grip], cube_pos, ee_pos, relative_pos, [distance]])
        return np.concatenate([
            qpos_arm, qvel_arm,             # 7 + 7 = 14
            [finger1_pos, finger2_pos],     # 1 + 1 = 2 (scalars!)
            relative_pos,                   # 3 
            [distance],                     # 1
            left_tip_pos, right_tip_pos,    # 3 + 3 = 6
            [left_contact, right_contact]   # 1 + 1 = 2 (binary contact, each is True or False)
        ])

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

        # # CURRICULUM: start easy, progress to hard
        # if not hasattr(self, 'curriculum_level'):
        #     self.curriculum_level = 0

        # if self.curriculum_level == 0:

        #     # easy cube position
        #     cube_x, cube_y = 0.5, 0.0

        # elif self.curriculum_level == 1:

        #     # bit harder
        #     cube_x = self.np_random.uniform(.045, 0.55)
        #     cube_y = self.np_random.uniform(-0.1, 0.1)

        # else:

        #     # hardest
        #     cube_x = self.np_random.uniform(0.4, 0.6)
        #     cube_y = self.np_random.uniform(-0.2, 0.2)

        cube_x = self.np_random.uniform(0.4, 0.6)
        cube_y = self.np_random.uniform(-0.2, 0.2)

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
        # print(self.model.joint("cube_joint").qposadr[0])
        self.data.qpos[self.model.joint("cube_joint").qposadr[0]:self.model.joint("cube_joint").qposadr[0]+3] = [cube_x, cube_y, 0.05]

        # step ahead
        mujoco.mj_forward(self.model, self.data)

        self.current_step = 0
        obs = self._get_obs()
        info = {}
        return obs, info

    def rewardValue(self, 
                    dist, 
                    finger1_pos, 
                    finger2_pos, 
                    cube_pos, 
                    ee_pos,                 # ← add this argument
                    left_tip_pos, 
                    right_tip_pos,
                    left_contact, 
                    right_contact, 
                    action):

        reward = 0.0

        # 1. Main drive: get close
        reward -= 1.5 * dist

        # 2. Horizontal centering (encourage being above the cube)
        horizontal_error = np.linalg.norm(cube_pos[:2] - ee_pos[:2])
        reward -= 2.0 * horizontal_error

        # 3. Prefer approaching from above when close
        if dist < 0.12 and ee_pos[2] > cube_pos[2] + 0.04:
            reward += 3.0

        # 4. Contact rewards
        if left_contact:
            reward += 5.0
        if right_contact:
            reward += 5.0
        if left_contact and right_contact:
            reward += 12.0

        # 5. Gripper closure quality
        finger_dist = np.linalg.norm(left_tip_pos - right_tip_pos)
        grip_closed = finger_dist < 0.025

        if left_contact and right_contact and grip_closed:
            reward += 18.0
        elif dist < 0.05 and not (left_contact or right_contact):
            reward -= 2.0

        # 6. Lifting (only after a proper grasp)
        if left_contact and right_contact and grip_closed and cube_pos[2] > 0.08:
            reward += 25.0 * (cube_pos[2] - 0.08)

        # 7. Success
        success = (cube_pos[2] > 0.15 and 
                grip_closed and 
                left_contact and 
                right_contact)
        if success:
            reward += 100.0
            self.success_count += 1

        # 8. Failure
        failure = (cube_pos[2] < 0.01 or 
                abs(cube_pos[0]) > 1.0 or 
                abs(cube_pos[1]) > 0.8)
        if failure:
            reward -= 50.0

        # 9. Small action penalty
        reward -= 0.0001 * np.sum(np.square(action[:7]))

        return reward, success, failure

    def _check_contact(self, body1_name, body2_name):
        """Check if two bodies are in contact by checking their geoms"""
        
        # Get all geom IDs for each body
        body1_id = self.model.body(body1_name).id
        body2_id = self.model.body(body2_name).id
        
        # Get geom address ranges for each body
        body1_geom_start = self.model.body_geomadr[body1_id]
        body1_geom_num = self.model.body_geomnum[body1_id]
        body1_geoms = set(range(body1_geom_start, body1_geom_start + body1_geom_num))
        
        body2_geom_start = self.model.body_geomadr[body2_id]
        body2_geom_num = self.model.body_geomnum[body2_id]
        body2_geoms = set(range(body2_geom_start, body2_geom_start + body2_geom_num))

        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            
            # Check if contact is between the two bodies
            if (geom1 in body1_geoms and geom2 in body2_geoms) or \
               (geom2 in body1_geoms and geom1 in body2_geoms):
                return 1.0

        return 0.0

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
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        qpos_arm = obs[0:7]
        qvel_arm = obs[7:14]
        finger1_pos = obs[14]
        finger2_pos = obs[15]
        relative_pos = obs[16:19]
        distance = obs[19]
        left_tip_pos = obs[20:23]
        right_tip_pos = obs[23:26]
        left_contact = obs[26]
        right_contact = obs[27]

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

        reward, success, failure = self.rewardValue(
                                                    distance, 
                                                    finger1_pos, 
                                                    finger2_pos, 
                                                    cube_pos,
                                                    ee_pos,
                                                    left_tip_pos, 
                                                    right_tip_pos, 
                                                    left_contact, 
                                                    right_contact, 
                                                    action
                                                    )
        
        terminated = success or failure 
        truncated = self.current_step >= self.max_episode_steps
        info = {"distance": dist, 
                "cube_height": cube_pos[2],
                "cube_x": cube_pos[0],
                "cube_y":cube_pos[1],
                "success": success,
                "failure": failure,
                "gripper": grip_command}

        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            else:
                self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

# Class Ends: write a test script

# env = PandaGrasper(render_mode="human")
# obs, info = env.reset()
# print("Obs shape:", obs.shape)
# n_steps = 300

# import time

# # Main Loop:

# for _ in range(n_steps):
#     action = env.action_space.sample()

#     # step the simulation with the next action
#     obs, reward, term, trunc, info = env.step(action)

#     env.render()

#     print(f"reward: {reward:.3f}, dist: {info['distance']:.3f}")
#     print(f"info: {info} \n")

#     if term or trunc:
#         print("Episode Ended: ", info)
#         break

#     time.sleep(2)

# env.close()