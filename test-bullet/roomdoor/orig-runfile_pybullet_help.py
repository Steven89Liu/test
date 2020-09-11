import gym
import pybullet
import numpy as np

class HumanoidDoorEnv(gym.Env):

  def __init__(self, xml_path='/home/shjliu/workspace/EPR/bullet3-new/test-bullet/roomdoor',
               num_solver_iterations=10, frame_skip=4, time_step=1/240., debug=0, render=0,
               actuator_lower_limit=(), actuator_upper_limit=(),
               robot_init_pos=(0, 0, 2), door_init_pos=(3, 0, 0), goal_pos=(3, 0, 0),
               door_controller_k=10**5, door_controller_d1=10**4, door_controller_d2=10**2,
               door_controller_th_handle=-30 * np.pi / 180, door_controller_th_door=5 * np.pi / 180,
               handle_controller_k=10**0.3, handle_controller_d=10**-1):

    # Set goal
    self.goal_pos = goal_pos
    self.robot_init_pos = robot_init_pos
    self.door_init_pos = door_init_pos

    # Camera parameters
    self._cam_dist = 5
    self._cam_yaw = 60
    self._cam_pitch = -66
    self._render_width = 320
    self._render_height = 240

    # Open physics client, load files and set parameters
    self.physicsClient_ID = -1
    self.xml_path = xml_path
    self.num_solver_iterations = num_solver_iterations
    self.frame_skip = frame_skip
    self.time_step = time_step
    self._set_physics_client(render)

    # Create relevant dictionaries
    body_name_2_joints = {}
    for body_id in range(pybullet.getNumBodies()):
      # Obtain name of specific body and decode it
      (bin_body_name, _) = pybullet.getBodyInfo(body_id)
      body_name = bin_body_name.decode("utf8")
      if body_name == "base_link":
        body_name = "door"

      # Get the amount of joints, their names and put them into a dictionary
      num_joints = pybullet.getNumJoints(body_id)
      joint_name_2_joint_id = {}
      if num_joints:
        for joint_id in range(num_joints):
          (_, bin_joint_name, joint_type, q_index, u_index, flags, joint_damping, joint_friction, joint_lower_limit, joint_upper_limit, joint_max_force, joint_max_velocity, link_name, joint_axis, parent_frame_pos, parent_frame_orn, parent_index) = pybullet.getJointInfo(body_id, joint_id)
          joint_name = bin_joint_name.decode("utf8")
          if joint_type == 4:
            num_joints -= 1
          else:
            joint_name_2_joint_id.update({joint_name: {"joint_id": joint_id, "joint_type": joint_type, "joint_damping": joint_damping,
                                                       "joint_lower_limit": joint_lower_limit, "joint_upper_limit": joint_upper_limit}})
      body_name_2_joints.update({body_name: {"body_id": body_id, "unique_body_id": pybullet.getBodyUniqueId(body_id), "body_name": body_name, "num_joints": num_joints, "joint_dict": joint_name_2_joint_id}})
    self.body_name_2_joints = body_name_2_joints

    # Important link ID's
    self.door_link_IDs = {
      'handle_inside': 7,
      'handle_outside': 9,
      'door_COM': 5}
    self.robot_link_IDs = {
      'torso': 0,
      'right_hand': 28,
      'left_hand': 34,
    }

    # Indicate names of controllable joints: ASSUMES ROBOT ROOT BODY IS NAMED 'robot' AND DOOR ROOT BODY 'door'
    self.robot_joints = [joint_name for joint_name in body_name_2_joints['robot']['joint_dict']]
    self.robot_joints_index = [body_name_2_joints['robot']['joint_dict'][joint_name]['joint_id'] for joint_name in body_name_2_joints['robot']['joint_dict']]
    self.door_joints = [joint_name for joint_name in body_name_2_joints['door']['joint_dict']]
    self.door_joints_index = [body_name_2_joints['door']['joint_dict'][joint_name]['joint_id'] for joint_name in body_name_2_joints['door']['joint_dict']]
    # joint_low = [body_name_2_joints['robot']['joint_dict'][joint_name]['joint_lower_limit'] for joint_name in body_name_2_joints['robot']['joint_dict']]
    # joint_high = [body_name_2_joints['robot']['joint_dict'][joint_name]['joint_upper_limit'] for joint_name in body_name_2_joints['robot']['joint_dict']]

    # Initialize door controllers
    self.door_controller_k = door_controller_k
    self.door_controller_d1 = door_controller_d1
    self.door_controller_d2 = door_controller_d2
    self.door_controller_th_handle = door_controller_th_handle
    self.door_controller_th_door = door_controller_th_door
    self.handle_controller_k = handle_controller_k
    self.handle_controller_d = handle_controller_d

    # Initialize cost function weight placeholders. Will be set in hidden method
    self.w_alive, self.w_hand_on_handle, self.w_at_target, self.w_move_door, self.w_move_handle, self.w_time, self.w_control = (0, 0, 0, 0, 0, 0, 0)
    self.set_reward_weights()

    # Initialize action and observation space
    if not actuator_lower_limit:
      actuator_lower_limit = -np.pi*np.ones_like(self.robot_joints_index)
      actuator_upper_limit = np.pi*np.ones_like(self.robot_joints_index)
    else:
      assert(len(self.robot_joints_index) == len(actuator_lower_limit) & len(self.robot_joints_index) == len(actuator_upper_limit))

    self.action_space = gym.spaces.Box(low=actuator_lower_limit, high=actuator_upper_limit, dtype=np.float32)
    obs, _, _, _ = self.step(self.action_space.sample(), debug=0)
    low = np.full(obs.shape, -float('inf'))
    high = np.full(obs.shape, float('inf'))
    self.observation_space = gym.spaces.Box(low=low, high=high, dtype=obs.dtype)

    self.obs_dict = {}
    self.reward_dict = {}

    # Debug
    if debug:
      self._debug_link_ids = []
      self._debug_lines_ids = []

      self._debug_links_text_init()
      self._debug_links_text_remove()

      self._debug_links_lines_init()
      self._debug_links_lines_remove()

  def reset(self):
    pybullet.resetBasePositionAndOrientation(self.body_name_2_joints['robot']['body_id'], self.robot_init_pos, [0, 0, 0, 1])
    pybullet.resetBaseVelocity(self.body_name_2_joints['robot']['body_id'], [0, 0, 0], [0, 0, 0])
    pybullet.resetBasePositionAndOrientation(self.body_name_2_joints['door']['body_id'], self.door_init_pos, [0, 0, 0, 1])
    pybullet.resetBaseVelocity(self.body_name_2_joints['door']['body_id'], [0, 0, 0], [0, 0, 0])
    for joint in self.body_name_2_joints['robot']['joint_dict'].values():
      pybullet.resetJointState(self.body_name_2_joints['robot']['body_id'], joint['joint_id'], 0, 0)
    for joint in self.body_name_2_joints['door']['joint_dict'].values():
      pybullet.resetJointState(self.body_name_2_joints['door']['body_id'], joint['joint_id'], 0, 0)

    obs_list, _ = self._get_observation()
    return obs_list

  def step(self, action, debug=0):
    # Apply policy action to robot
    # pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['robot']['body_id'],
    #                                    jointIndices=self.robot_joints_index,
    #                                    controlMode=pybullet.VELOCITY_CONTROL,
    #                                    forces=np.zeros_like(self.action_space.low))
    # pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['robot']['body_id'],
    #                                    jointIndices=self.robot_joints_index,
    #                                    controlMode=pybullet.TORQUE_CONTROL,
    #                                    forces=np.maximum(np.minimum(action, self.action_space.high), self.action_space.low))
    pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['robot']['body_id'],
                                       targetPositions=action,
                                       jointIndices=self.robot_joints_index,
                                       controlMode=pybullet.POSITION_CONTROL,
                                       forces=10*np.ones_like(self.action_space.high))
    # Keep door closed if handle is not pressed
    ((q_door_hinge, v_door_hinge, _, applied_torque_door_hinge),
     (q_handle_hinge, v_handle_hinge, _, applied_torque_handle_hinge)) = pybullet.getJointStates(self.body_name_2_joints['door']['body_id'], self.door_joints_index)
    handle_pushed = q_handle_hinge <= self.door_controller_th_handle
    door_closed = np.abs(q_door_hinge) <= self.door_controller_th_door
    if not handle_pushed and door_closed:
      door_actuation = -self.door_controller_k * q_door_hinge - self.door_controller_d1 * v_door_hinge
    else:
      door_actuation = -self.door_controller_d2 * v_door_hinge
    handle_actuation = -self.handle_controller_k * q_handle_hinge - self.handle_controller_d * v_handle_hinge
    # pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['door']['body_id'],
    #                                    jointIndices=self.door_joints_index,
    #                                    controlMode=pybullet.VELOCITY_CONTROL,
    #                                    forces=np.array([0, 0.]))
    # pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['door']['body_id'],
    #                                    jointIndices=self.door_joints_index,
    #                                    controlMode=pybullet.TORQUE_CONTROL,
    #                                    forces=np.array([door_actuation, 0.]))
    # pybullet.setJointMotorControl2(bodyIndex=self.body_name_2_joints['door']['body_id'], jointIndex=self.door_joints_index[1],
    #                                controlMode=pybullet.POSITION_CONTROL, targetPosition=0)
    # print("Applied %.3f [Nm] on handle" % applied_torque_handle_hinge)
    pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['door']['body_id'],
                                       jointIndices=self.door_joints_index,
                                       controlMode=pybullet.VELOCITY_CONTROL, forces=[0, 0])
    pybullet.setJointMotorControlArray(bodyIndex=self.body_name_2_joints['door']['body_id'],
                                       jointIndices=self.door_joints_index,
                                       controlMode=pybullet.TORQUE_CONTROL, forces=[door_actuation, handle_actuation])
    # Advance the simulation
    pybullet.stepSimulation()

    # Calculate reward and observation
    obs_list, self.obs_dict = self._get_observation()
    reward, self.reward_dict = self._get_reward(action, self.obs_dict)

    # Done condition
    done = (self.obs_dict['torso_pos'][2] <= 0.0) | (self.obs_dict['torso_pos'][2] >= 3.0)
    info = {}

    # Debug
    if debug:
      self._debug_links_text()

    return obs_list, reward, done, info

  def render(self, mode="human"):
    self._set_physics_client(render=(1 if mode == 'human' else 0))

  def close(self):
    pybullet.disconnect()
    pass

  def set_reward_weights(self, w_alive=10.0, w_hand_on_handle=-1.0, w_at_target=-2., w_move_door=0.01, w_move_handle=0.01, w_time=-0.01, w_control=0.1):
    self.w_alive = w_alive
    self.w_hand_on_handle = w_hand_on_handle
    self.w_at_target = w_at_target
    self.w_move_door = w_move_door
    self.w_move_handle = w_move_handle
    self.w_time = w_time
    self.w_control = w_control

  def _get_reward(self, action, observation):
    # Robot should stay alive
    r_alive = self.w_alive

    # One of the hands should go to the door handle: r_1 = min_hands(||x_hand - x_handle||)
    r_hand_handle_min = -4.
    d_lhand_handle = observation['left_hand_pos'] - observation['handle_inside_pos']
    d_rhand_handle = observation['right_hand_pos'] - observation['handle_inside_pos']
    d_hand_handle = min(np.linalg.norm(d_lhand_handle), np.linalg.norm(d_rhand_handle))
    r_hand_handle = max(self.w_hand_on_handle * d_hand_handle, r_hand_handle_min)  # Avoid excessive negative penalties

    # Robot should go to target
    r_at_target_min = -6.
    d_torso_goal = observation['torso_pos'] - self.goal_pos
    d_torso_goal = np.square(d_torso_goal[0:1]).sum()  # z-coordinate does not matter
    r_at_target = max(self.w_at_target * d_torso_goal, r_at_target_min)

    # Robot should move the door to open it
    r_move_door = self.w_move_door * np.abs(observation['door_hinge_pos'])

    # Robot should move handle to open door
    r_move_handle = self.w_move_handle * observation['handle_hinge_pos']

    # Robot should do this as fast as possible
    r_time = self.w_time

    # Robot should use minimal actuation (to stand in most favourable pose in the end)
    r_ctrl = self.w_control * np.square(action).sum()

    r = self.w_alive + r_hand_handle + r_at_target + r_move_door + r_move_handle + r_time + r_ctrl
    return r[0], dict(reward_alive=r_alive, reward_hand_on_handle=r_hand_handle, reward_at_target=r_at_target,
                      reward_move_door=r_move_door, reward_move_handle=r_move_handle, reward_time=r_time,
                      reward_control=r_ctrl)

  def _get_observation(self):
    # Obtain relevant quantities: Links
    (handle_inside, handle_outside, door) = pybullet.getLinkStates(self.body_name_2_joints['door']['body_id'],
                                                                   linkIndices=[self.door_link_IDs['handle_inside'], self.door_link_IDs['handle_outside'], self.door_link_IDs['door_COM']],
                                                                   computeLinkVelocity=1)

    (torso, right_hand, left_hand) = pybullet.getLinkStates(self.body_name_2_joints['robot']['body_id'],
                                                            linkIndices=[self.robot_link_IDs['torso'], self.robot_link_IDs['right_hand'], self.robot_link_IDs['left_hand']],
                                                            computeLinkVelocity=1)

    #  Obtain relevant quantities: Joints
    (door_hinge, handle_hinge) = pybullet.getJointStates(self.body_name_2_joints['door']['body_id'], self.door_joints_index)
    robot_joints = pybullet.getJointStates(self.body_name_2_joints['robot']['body_id'], self.robot_joints_index)

    # Put them in a dictionary
    obs_dict = {'handle_inside_pos': np.array(handle_inside[0]),
                # 'handle_inside_vel': np.array(handle_inside[6]),
                'handle_outside_pos': np.array(handle_outside[0]),
                # 'handle_outside_vel': np.array(handle_outside[6]),
                'door_pos': np.array(door[0]),
                # 'door_vel': np.array(door[6]),
                'torso_pos': np.array(torso[0]),
                # 'torso_vel': np.array(torso[6]),
                'right_hand_pos': np.array(right_hand[0]),
                # 'right_hand_vel': np.array(right_hand[6]),
                'left_hand_pos': np.array(left_hand[0]),
                # 'left_hand_vel': np.array(left_hand[6]),
                'door_hinge_pos': np.array([door_hinge[0]]),
                # 'door_hinge_vel': np.array([door_hinge[1]]),
                'handle_hinge_pos': np.array([handle_hinge[0]]),
                # 'handle_hinge_vel': np.array([handle_hinge[1]]),
                'robot_joint_pos': np.array([robot_joints[i][0] for i in range(len(robot_joints))]),
                # 'robot_joint_vel': np.array([robot_joints[i][1] for i in range(len(robot_joints))])
                }

    # Flatten into raw observation list
    obs_list = np.array([entry for obs in obs_dict.values() for entry in obs])

    return obs_list, obs_dict

  def _debug_links_text_init(self):
    for dict_entry in self.body_name_2_joints:
      idd = self.body_name_2_joints[dict_entry]['body_id']
      i = 0
      while True:
        result = pybullet.getLinkState(idd, i)
        if result is None:
          break
        pos = result[0]
        iddd = pybullet.addUserDebugText('L(%d, %d)' % (idd, i), pos, [0., 0., 0.])
        self._debug_link_ids += [iddd]
        i += 1

  def _debug_links_text(self):
    for dict_entry in self.body_name_2_joints:
      idd = self.body_name_2_joints[dict_entry]['body_id']
      i = 0
      while i == 0:
        result = pybullet.getLinkState(idd, i)
        if result is None:
          break
        pos = result[0]
        pybullet.addUserDebugText(text='(%d, %d)' % (idd, i), textPosition=pos, textColorRGB=[0., 0., 0.], replaceItemUniqueId=self._debug_link_ids[i])
        i += 1

  def _debug_links_text_remove(self):
    for idd in self._debug_link_ids:
      pybullet.removeUserDebugItem(idd)

  def _debug_links_lines_init(self):
    (handle_inside, handle_outside) = pybullet.getLinkStates(self.body_name_2_joints['door']['body_id'], linkIndices=[self.door_link_IDs['handle_inside'], self.door_link_IDs['handle_outside']])
    (left_hand, right_hand) = pybullet.getLinkStates(self.body_name_2_joints['robot']['body_id'], linkIndices=[self.robot_link_IDs['left_hand'], self.robot_link_IDs['right_hand']])
    self._debug_lines_ids += [pybullet.addUserDebugLine(lineFromXYZ=handle_inside[0], lineToXYZ=left_hand[0], lineColorRGB=[0., 0., 0.])]
    self._debug_lines_ids += [pybullet.addUserDebugLine(lineFromXYZ=handle_inside[0], lineToXYZ=right_hand[0], lineColorRGB=[0., 0., 0.])]
    self._debug_lines_ids += [pybullet.addUserDebugLine(lineFromXYZ=handle_outside[0], lineToXYZ=left_hand[0], lineColorRGB=[0., 0., 0.])]
    self._debug_lines_ids += [pybullet.addUserDebugLine(lineFromXYZ=handle_outside[0], lineToXYZ=right_hand[0], lineColorRGB=[0., 0., 0.])]

  def _debug_links_lines_remove(self):
    for idd in self._debug_lines_ids:
      pybullet.removeUserDebugItem(idd)

  def _set_physics_client(self, render=0):
    if self.physicsClient_ID != -1:
      pybullet.disconnect(self.physicsClient_ID)
    self.physicsClient_ID = pybullet.connect(pybullet.GUI) if render else pybullet.connect(pybullet.DIRECT)

    # Load door
    pybullet.setAdditionalSearchPath(self.xml_path)
    pybullet.loadURDF(fileName="room_with_door.urdf", basePosition=self.door_init_pos, physicsClientId=self.physicsClient_ID,
                      flags=pybullet.URDF_USE_INERTIA_FROM_FILE | pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
    # Load humanoid
    pybullet.loadURDF(fileName='humanoid.urdf', basePosition=self.robot_init_pos, physicsClientId=self.physicsClient_ID,
                      flags=pybullet.URDF_USE_INERTIA_FROM_FILE | pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

    # Set and store  physics parameters
    pybullet.setDefaultContactERP(.90)
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setPhysicsEngineParameter(fixedTimeStep=self.time_step*self.frame_skip, numSolverIterations=self.num_solver_iterations, numSubSteps=self.frame_skip)


env = HumanoidDoorEnv()
done = 0
env.reset()
env.render(mode='human')
for _ in range(10000):
  _, _, done, _ = env.step(env.action_space.sample())
  if done:
    obs = env.reset()
