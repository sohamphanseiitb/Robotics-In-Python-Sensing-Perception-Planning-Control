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

        ## approximately joint limits of the Panda, (in radians)
        # earlier were -2.9 and 2.9, now changed to -0.3 and 0.3 to accomodate for additional inputs
        # coming from base controller
        self.action_space = spaces.Box(low=np.array([-0.2]*7 + [-0.3]),
                                       high=np.array([0.2]*7 + [0.3]),
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
        self.grasp_state = "approach"   # "approach" -> "closed" -> "lift"

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

        # set state machine
        self.grasp_state = "approach"

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
                    ee_pos,
                    left_tip_pos, 
                    right_tip_pos,
                    left_contact, 
                    right_contact, 
                    action,
                    hand_z_axis):                 # new argument

        reward = 0.0

        # 1. Get close
        reward -= 1.5 * dist

        # 2. Horizontal centering
        horizontal_error = np.linalg.norm(cube_pos[:2] - ee_pos[:2])
        reward -= 2.5 * horizontal_error

        # 3. Approach from above
        if dist < 0.15 and ee_pos[2] > cube_pos[2] + 0.05:
            reward += 4.0

        # 4. Orientation: prefer top-down (hand z-axis pointing down)
        if dist < 0.18:
            alignment = np.dot(hand_z_axis, np.array([0.0, 0.0, -1.0]))
            reward += 5.0 * max(alignment, 0.0)

        # fingers are grasping the cube from adjacent faces which is a problem  
        midpoint = 0.5 * (left_tip_pos + right_tip_pos)
        midpoint_error = np.linalg.norm(midpoint - cube_pos)
        reward -= 6.0 * midpoint_error

        # 5. Contact (kept modest)
        if left_contact:
            reward += 1.5
        if right_contact:
            reward += 1.5
        if left_contact and right_contact:
            reward += 6.0

        # 6. Gripper closure
        finger_dist = np.linalg.norm(left_tip_pos - right_tip_pos)
        grip_closed = finger_dist < 0.025

        if left_contact and right_contact and grip_closed:
            reward += 8.0
        elif dist < 0.04 and not (left_contact or right_contact):
            reward -= 3.0                         # milder than -10

        # 7. Lifting
        if left_contact and right_contact and grip_closed and cube_pos[2] > 0.08:
            reward += 60.0 * (cube_pos[2] - 0.08)

        # 8. Success
        success = (cube_pos[2] > 0.15 and 
                grip_closed and 
                left_contact and 
                right_contact)
        if success:
            reward += 150.0
            self.success_count += 1

        # 9. Failure
        failure = (cube_pos[2] < 0.01 or 
                abs(cube_pos[0]) > 1.0 or 
                abs(cube_pos[1]) > 0.8)
        if failure:
            reward -= 40.0

        # 10. Action penalty
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

    def classical_controller(self, data, cube_pos, ee_pos):
        """
        Classical controller that aims for a top-down antipodal grasp.
        Gripper convention: 0.0 = closed, 1.0 = open
        """
        # Current state of franka_panda
        q = data.qpos[0:7].copy()

        # metrics
        dist = np.linalg.norm(ee_pos - cube_pos) # distance between end effector and cube
        horizontal_error = np.linalg.norm((cube_pos - ee_pos)[:2]) # horizontaal distance between end effector and cube
        height_from_cube = abs(ee_pos[2] - cube_pos[2])

        # Fingertip positions (sites)
        left_tip  = data.site("left_fingertip").xpos.copy()
        right_tip = data.site("right_fingertip").xpos.copy()
        midpoint_pos  = 0.5 * (left_tip + right_tip) # position of midpoint of two fingers
        midpoint_error = np.linalg.norm(midpoint_pos - cube_pos)

        # ---------- State machine ----------
        # approach -> closed -> lift
        if self.grasp_state == "approach" and midpoint_error < 0.02:
            self.grasp_state = "closed"

        if self.grasp_state == "closed":
            self.grasp_state = "lift"

        # ---------- Target + gripper depending on state ----------
        if self.grasp_state == "lift":
            # Keep gripper closed and go straight up
            target_pos = midpoint_pos.copy()
            target_pos[2] = cube_pos[2] + 0.18
            grip = 0.0
        else:
            # Phase 1: Coarse adjustment

            ## Phase 1.1 First Adjust height and bring midpoint_pos 5 cm over the cube
            if height_from_cube > 0.2:
                target_pos = midpoint_pos.copy()
                target_pos[2] = cube_pos[2] + 0.1

            ## Phase 1.2 Vertical adjustment complete: arm hovering 10 cm above the cube
            ## Adjust horizontal error
            elif horizontal_error > 0.05:
                target_pos = midpoint_pos.copy()
                target_pos[:2] = cube_pos[:2]

            ## Phase 1.3 Micro Adjustment: both vertical and horizontal coarse adjustment done
            ## now fine tuning, adjust midpoint of fingers to co-incide with cube center
            else:
                # Final grasp height
                target_pos = cube_pos + np.array([0.0, 0.0, 0.0005])

            # ---------- Gripper logic ----------
            # Keep gripper open until arm is close enough
            if midpoint_error > 0.005: # 2cm 
                grip = 1.0          # open
            else:
                grip = 0.0          # close

        # Amplify position error
        pos_error = target_pos - midpoint_pos
        pos_error[0] *= 2.2
        pos_error[1] *= 2.2

        # ---------- 2. Differential IK (damped least squares) ----------
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, data, jacp, None, self.hand_body_id)
        J = jacp[:, :7]

        damping = 1e-2
        JTJ = J.T @ J + damping * np.eye(7)
        dq = np.linalg.solve(JTJ, J.T @ pos_error)

        error_norm = np.linalg.norm(pos_error)
        alpha = np.clip(0.75 * error_norm, 0.2, 0.7)

        # each joint has different limits: use self.model.jnt_range
        """
        J1: -2.9 to 2.9
        J2: -1.76 to 1.76
        J3: -2.9 to 2.9
        J4: -3.07 to 0
        J5: -2.9 tp 2.9
        J6: 0 to 3.75
        J7: -2.9 to 2.9
        """
        q_target = np.zeros_like(q)
        q_target[0] = np.clip(q[0] + alpha * dq[0], -2.8, 2.8)
        q_target[1] = np.clip(q[1] + alpha * dq[1], -1.6, 1.6)
        q_target[2] = np.clip(q[2] + alpha * dq[2], -2.8, 2.8)
        q_target[3] = np.clip(q[3] + alpha * dq[3], -3.0, -0.2)
        q_target[4] = np.clip(q[4] + alpha * dq[4], -2.8, 2.8)
        q_target[5] = np.clip(q[5] + alpha * dq[5], 0.1, 3.6)
        q_target[6] = np.clip(q[6] + alpha * dq[6], -2.8, 2.8)

        return np.concatenate([q_target, [grip]])
    # write a function to drive the observe-act-observe outer loop
    def step(self, policy_action):

        # clip policy action
        policy_action = np.clip(policy_action, 
                                self.action_space.low, 
                                self.action_space.high)

        # testing
        policy_action = np.zeros_like(policy_action)

        # compute action from base controller
        base_action = self.classical_controller(self.data,
                                                self.data.body(self.cube_body_id).xpos,
                                                self.data.body(self.hand_body_id).xpos)

        final_action = base_action + policy_action
        final_action[:7] = np.clip(final_action[:7], -2.9, 2.9)
        final_action[7] = np.clip(final_action[7], 0.0, 1.0)
        
        self.current_step += 1

        # split action
        joint_targets = final_action[:7]
        grip_command = final_action[7]   # we are normalizing action between 0, 1 hence [0, 1] expected here

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

        hand_rot = self.data.body(self.hand_body_id).xmat.reshape(3, 3)
        hand_z_axis = hand_rot[:, 2]

        reward, success, failure = self.rewardValue(distance, 
                                                    finger1_pos, 
                                                    finger2_pos, 
                                                    cube_pos,
                                                    ee_pos,
                                                    left_tip_pos, 
                                                    right_tip_pos, 
                                                    left_contact, 
                                                    right_contact, 
                                                    final_action,
                                                    hand_z_axis)                                                    
        
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

env = PandaGrasper(render_mode="human")
obs, info = env.reset()
print("Obs shape:", obs.shape)
n_steps = 300

import time
for _ in range(n_steps):
    action = env.action_space.sample()

    # step the simulation with the next action
    obs, reward, term, trunc, info = env.step(action)
    env.render()

    print(f"reward: {reward:.3f}, dist: {info['distance']:.3f}")
    print(f"info: {info} \n")

    if term or trunc:
        print("Episode Ended: ", info)
        break
    time.sleep(0.2)
env.close()