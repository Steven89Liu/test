import pybullet as p
import time
import numpy as np
import pybullet_data
import pkg_resources
import sys
from collections import OrderedDict, namedtuple


################################################################################
#   Constants
################################################################################

TROLLEY_SIM_MAX = 1.5
TROLLEY_SIM_MIN = 0.0
TROLLEY_CRANE_MAX = 25.67
TROLLEY_CRANE_MIN = 3.9
SIM_TRACK_RADIUS = 1.
REAL_TRACK_RADIUS = 25.3746
GANTRY_VELOCITY_MIN = 0.0039726027397260265

GANTRY_KAI_TO_SIM = SIM_TRACK_RADIUS/REAL_TRACK_RADIUS
TROLLEY_KAI_TO_SIM = (TROLLEY_SIM_MAX - TROLLEY_SIM_MIN) / (TROLLEY_CRANE_MAX - TROLLEY_CRANE_MIN)


class Simulation(object):
    ''' Minimalistic Simulation class to demonstration drift issue.

        Param:
            gui(bool): enables/disables pybullet gui
            gravity(tuple): sets gravitiy in simulator
            use_realistic_speed(bool): enable/disable conversions between
                                       simulator and real world application


    '''
    def __init__(
        self,
        gui=True,
        gravity=(0, 0, -9),
        use_realistic_speed=True,
        en_sway=True,

    ):
        self.use_realistic_speed = use_realistic_speed
        self.en_sway = en_sway

        if gui:
            self.connection_mode = p.GUI
        else:
            self.connection_mode = p.DIRECT

        self.cnt = 0
        physicsClient = p.connect(self.connection_mode)

        # do not use stupid urdf caching which shits the bed whenever urdf is
        # changed
        p.setPhysicsEngineParameter(enableFileCaching=0, numSolverIterations=10) #, useSplitImpulse=0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setRealTimeSimulation(enableRealTimeSimulation=1)

        # disable debug windows
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        self.gravity = gravity
        p.setGravity(*self.gravity)
        planeId = p.loadURDF("plane.urdf")  # noqa

        cube_start_pos = [0, 0, 0]
        cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        if self.en_sway:
            crane_urdf_path = "models/crane.urdf"
        else:
            crane_urdf_path = "models/crane_nosway.urdf"

        print(crane_urdf_path)

        logstash_urdf_path = pkg_resources.resource_filename(
            __name__, "models/logs.urdf"
        )

        self.crane_body_id = p.loadURDF(
            crane_urdf_path, cube_start_pos, cube_start_orientation #,flags=p.URDF_USE_SELF_COLLISION#, useFixedBase=1
        )

        self.log_stash = p.loadURDF(
            logstash_urdf_path, cube_start_pos, cube_start_orientation
        )
        self.num_joints = p.getNumJoints(self.crane_body_id)

        for joint in range(0, self.num_joints):
            p.setCollisionFilterPair(self.crane_body_id, self.log_stash, joint, 1, 1)

        # link name -> link index
        self._link_map = {}
        # joint name -> joint index
        self._joint_map = {}



        ####################################################################################
        #  print list of all joints and create mappings name -> index for links and joints #
        ####################################################################################
        print(
            "List of all joints (Joint Index is the Link Index belonging to Child Link Name)"
        )
        header_template = (
            "{joint_idx:^20}|{joint_name:^40}|{link_name:^40}|{parent_idx:^20}"
        )
        header = header_template.format(
            joint_idx="Joint Index",
            joint_name="Joint Name",
            link_name="Child Link Name",
            parent_idx="Parent Link Index",
        )
        print(header)
        print("-" * len(header))
        for joint in range(self.num_joints):
            joint_info = p.getJointInfo(self.crane_body_id, joint)
            joint_name = joint_info[1].decode("utf-8")
            link_name = joint_info[12].decode("utf-8")
            parent_idx = joint_info[16]
            print("{joint:^20}|{joint_name:^40}|{link_name:^40}|{parent_idx:^20}".format(joint=joint, joint_name=joint_name,link_name=link_name,parent_idx=parent_idx))

            self._link_map[link_name] = joint
            self._joint_map[joint_name] = joint

        if self.en_sway:
            p.setJointMotorControl2(
                self.crane_body_id,
                self._joint_map["trolley_to_sway_x"],
                p.POSITION_CONTROL,
                force=1,
            )

            p.setJointMotorControl2(
                self.crane_body_id,
                self._joint_map["sway_x_to_sway_y"],
                p.POSITION_CONTROL,
                force=1,
            )



            p.changeDynamics(
                self.crane_body_id,
                self._link_map["sway_link_x"],
                lateralFriction=1,
                spinningFriction=1,
                rollingFriction=1,
                contactStiffness=10000,
                contactDamping=0,
                frictionAnchor=10000,
            )


            p.changeDynamics(
                self.crane_body_id,
                self._link_map["sway_link_y"],
                lateralFriction=1,
                spinningFriction=1,
                rollingFriction=1,
                contactStiffness=10000,
                contactDamping=0,
                frictionAnchor=10000,
            )



        ### for issue demonstration purpose only
        ### normally action generation is handled via a zmq connection
        self.joint_names = ["gantry", "trolley", "hoist", "grapple", "tines"]
        self.current_speed = OrderedDict(
            zip(self.joint_names, [0.0, 0.0, 0.0, 0.0, 0.0])
        )
        self.desired_speed = OrderedDict(
            zip(self.joint_names, [0.0, 0.0, 0.0, 0.0, 0.0])
        )
        self.speed_limit = OrderedDict(
            zip(self.joint_names, [2.03, 0.76, 0.37, 1.0, 1.0])
        )
        self.speed_step = OrderedDict(
            zip(self.joint_names, [0.01, 0.005, 0.005, 1.0, 1.0])
        )

        self.last_known_encoders = None

        self.step_counter = 0


    def _get_action(self):
        t0 = time.time()

        # for drift demonstration: accelerate for n steps, then de-accelerate
        if self.cnt < 500:
            self.desired_speed = OrderedDict(
                zip(self.joint_names, [2.03, 0.0, 0.0, 0.0, -1])
            )
        else:
            self.desired_speed = OrderedDict(
                zip(self.joint_names, [0.0, 0.0, 0.0, 0.0, 0.0])
            )



        desired = np.fromiter(self.desired_speed.values(), dtype=np.float)
        current = np.fromiter(self.current_speed.values(), dtype=np.float)
        speed_step = np.fromiter(self.speed_step.values(), dtype=np.float)

        I = (np.abs(desired - current) >= speed_step) * 1.0
        current += (
            I * speed_step * np.sign(desired - current)
        )  # make speed step for values out of range
        current *= I  # clear values in range of speed limit
        current += (
            1.0 - I
        ) * desired  # set values in range of speed limit to desired

        self.current_speed = OrderedDict(zip(self.joint_names, current))

        # action = autocrane_pb2.Action()
        # >>> Color = namedtuple('Color', 'red green blue')
        action = namedtuple('action', 'gantry trolley hoist grapple tines')
        for k in ["gantry", "trolley", "hoist"]:
            setattr(action, k, self.current_speed[k])
        setattr(action, "grapple", int(self.current_speed["grapple"]))
        setattr(
            action,
            "tines",
            0
            if self.current_speed["tines"] == 0
            else (1 if self.current_speed["tines"] == -1 else 2),
        )
        print("=" * 200)
        print("Cnt: {} Sending command {}".format(self.cnt,dict(self.current_speed)))

        self.cnt += 1
        time.sleep(
            max(0, 1 / 30 - (time.time() - t0))
        )  # sleep for remaining time of 30Hz-cycle


        return action


    def _act(self, action):
        if self.use_realistic_speed:
            gantry_velocity = np.clip(action.gantry, -2.04, 2.04) * GANTRY_KAI_TO_SIM
            trolley_velocity = np.clip(action.trolley, -1, 1) * TROLLEY_KAI_TO_SIM

        else:
            gantry_velocity = np.clip(action.gantry, -2.04, 2.04) * 1
            trolley_velocity = np.clip(action.trolley, -1, 1) * 1


        rope_velocity = np.clip(action.hoist, -1, 1)
        grapple_rot_velocity = np.clip(action.grapple, -1, 1)
        jaw_force = action.tines
        ##########################################################################################
        #  Since pybullet does not respect urdf limits, we manually limit the
        #  joint. note that the limits for some weird reason are not hit
        #  exactly when the joints move fast. #
        ##########################################################################################
        # limit hoist
        if self.en_sway:
            hoist_pos, *_ = p.getJointState(
                self.crane_body_id, self._joint_map["sway_y_to_hoist"]
            )
            trolley_hoist_info = p.getJointInfo(
                self.crane_body_id, self._joint_map["sway_y_to_hoist"]
            )
        else:
            hoist_pos, *_ = p.getJointState(
                self.crane_body_id, self._joint_map["trolley_to_hoist"]
            )
            trolley_hoist_info = p.getJointInfo(
                self.crane_body_id, self._joint_map["trolley_to_hoist"]
            )

        # limit trolley
        trolley_pos, *_ = p.getJointState(
            self.crane_body_id, self._joint_map["gantry_to_trolley"]
        )
        gantry_trolley_info = p.getJointInfo(
            self.crane_body_id, self._joint_map["gantry_to_trolley"]
        )


        hoist_lower_limit, hoist_upper_limit = trolley_hoist_info[8:10]
        if hoist_pos <= hoist_lower_limit:
            rope_velocity = max(rope_velocity, 0)
        if hoist_pos >= hoist_upper_limit:
            rope_velocity = min(rope_velocity, 0)


        trolley_lower_limit, trolley_upper_limit = gantry_trolley_info[8:10]
        if trolley_pos <= trolley_lower_limit:
            trolley_velocity = max(trolley_velocity, 0)
        if trolley_pos >= trolley_upper_limit:
            trolley_velocity = min(trolley_velocity, 0)



        #### End limit #####################################################
        print("velocity gantry=",gantry_velocity, "trolley=", trolley_velocity, "rope=", rope_velocity, "grapple=", grapple_rot_velocity, "jam=", jaw_force)
        if self.en_sway:
            p.setJointMotorControlArray(
                self.crane_body_id,
                [
                    self._joint_map["tracks_to_gantry"],
                    self._joint_map["hub_to_gantry"],
                    self._joint_map["gantry_to_trolley"],
                    self._joint_map["sway_y_to_hoist"],
                    self._joint_map["grapple_base_to_grapple_body"],
                    self._joint_map["grapple_body_to_left_grapple_jaw"],
                    self._joint_map["grapple_body_to_right_grapple_jaw"],
                ],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[
                    0,
                    gantry_velocity,
                    trolley_velocity,
                    rope_velocity,
                    grapple_rot_velocity,
                    -jaw_force,
                    jaw_force,
                ],
            )
        else:
            p.setJointMotorControlArray(
                self.crane_body_id,
                [
                    self._joint_map["hub_to_gantry"],
                    self._joint_map["gantry_to_trolley"],
                    self._joint_map["trolley_to_hoist"],
                    self._joint_map["grapple_base_to_grapple_body"],
                    self._joint_map["grapple_body_to_left_grapple_jaw"],
                    self._joint_map["grapple_body_to_right_grapple_jaw"],
                ],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[
                    gantry_velocity,
                    trolley_velocity,
                    rope_velocity,
                    grapple_rot_velocity,
                    -jaw_force,
                    jaw_force,
                ],
            )



    def update_encoders(self):
        """ Get encoder states of joints """


        if self.en_sway:
            # directly accessing joint states instead of calculating transforms
            states = p.getJointStates(
                bodyUniqueId = self.crane_body_id,
                jointIndices = [
                    self._joint_map["hub_to_gantry"],
                    self._joint_map["gantry_to_trolley"],
                    self._joint_map["trolley_to_sway_x"],
                    self._joint_map["sway_x_to_sway_y"],
                    self._joint_map["sway_y_to_hoist"],
                    self._joint_map["grapple_base_to_grapple_body"],
                    self._joint_map["grapple_body_to_left_grapple_jaw"],
                    self._joint_map["grapple_body_to_right_grapple_jaw"],
                    self._joint_map["tracks_to_gantry"],
                ]
            )
            gantry_position = states[0][0]
            trolley_position = states[1][0]
            hoist_position = states[4][0]
            sway_angle_x = states[2][0]
            sway_angle_y = states[3][0]
            grapple_base_rotation = states[5][0] % (2*np.pi)
            tines_rotation = states[6][0]
            tines_rotationR = states[7][0]
            hub_pos =  states[8][0]

            print("hub angle: {}".format(hub_pos))
            print("Gantry angle: {}".format(gantry_position))
            print("Trolley pos: {}".format(trolley_position))
            print("Sway angle X: {}".format(sway_angle_x))
            print("Sway angle Y: {}".format(sway_angle_y))
            print("Hoist pos: {}".format(hoist_position))
            print("Grapple rotation angle: {}".format(grapple_base_rotation))
            print("Tines angle: {}".format(tines_rotation))
            print("Tines angle R: {}".format(tines_rotationR))
            '''
            if tines_rotationR < -0.6:
                if self.cnt < 600:
                    input()
                self.cnt = 600
            '''
        else:
            states = p.getJointStates(
                bodyUniqueId = self.crane_body_id,
                jointIndices = [
                    self._joint_map["hub_to_gantry"],
                    self._joint_map["gantry_to_trolley"],
                    self._joint_map["trolley_to_hoist"],
                    self._joint_map["grapple_base_to_grapple_body"],
                    self._joint_map["grapple_body_to_left_grapple_jaw"],
                    self._joint_map["grapple_body_to_right_grapple_jaw"],
                    self._joint_map["tracks_to_gantry"],
                ]
            )

            gantry_position = states[0][0]
            trolley_position = states[1][0]
            hoist_position = states[4][0]
            grapple_base_rotation = states[3][0] % (2*np.pi)
            tines_rotation = states[5][0]
            hub_pos =  states[6][0]

            print("hub angle: {}".format(hub_pos))
            print("Gantry angle: {}".format(gantry_position))
            print("Trolley pos: {}".format(trolley_position))
            print("Hoist pos: {}".format(hoist_position))
            print("Grapple rotation angle: {}".format(grapple_base_rotation))
            print("Tines angle: {}".format(tines_rotation))

    def step(self):
        self.update_encoders()

        action = self._get_action()
        self._act(action)

    def run(self):
        ## ------- profiling
        # turn on to get profiling info
        profile = False
        ## ------- profiling

        stop = False

        ## ------- profiling
        if profile:

            profile = cProfile.Profile()
            profile.enable()
        ## ------- profiling

        # main loop
        while not stop:
            try:
                if self.connection_mode == p.DIRECT:
                    p.stepSimulation()
                self.step()
                #p.stepSimulation()
            except KeyboardInterrupt:
                stop = True

        ## ------- profiling
        if profile:
            profile.disable()
            s = io.StringIO()
            sortby = "cumulative"
            ps = pstats.Stats(profile, stream=s).sort_stats(sortby)
            ps.print_stats()
            print(s.getvalue())
        ## ------- profiling

    def shutdown(self):
        p.disconnect
