import math
import struct
import time

TIME_STEP = 64
ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)


class RCJSoccerRobot:
    def __init__(self, robot):
        self.robot = robot
        self.name = self.robot.getName()
        self.team = self.name[0]
        self.player_id = int(self.name[1])

        self.receiver = self.robot.getDevice("supervisor receiver")
        self.receiver.enable(TIME_STEP)

        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TIME_STEP)

        self.ball_receiver = self.robot.getDevice("ball receiver")
        self.ball_receiver.enable(TIME_STEP)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        self.compass = self.robot.getDevice("compass")
        self.compass.enable(TIME_STEP)

        self.sonar_left = self.robot.getDevice("distancesensor left")
        self.sonar_left.enable(TIME_STEP)
        self.sonar_right = self.robot.getDevice("distancesensor right")
        self.sonar_right.enable(TIME_STEP)
        self.sonar_front = self.robot.getDevice("distancesensor front")
        self.sonar_front.enable(TIME_STEP)
        self.sonar_back = self.robot.getDevice("distancesensor back")
        self.sonar_back.enable(TIME_STEP)

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float("+inf"))
        self.right_motor.setPosition(float("+inf"))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.heading = 0
        self.ultrasonic_data = dict()
        self.team_data = dict()

        self.time_step = 0
        self.ball_pos_arr = []
        self.robot_pos_arr = []
        self.time_steps_arr = []
        self.temp_ball_speeds = []
        self.temp_robot_speeds = []
        self.dist_arr = []
        self.right_wheel_vel = 0
        self.left_wheel_vel = 0

        self.moving_to_x = False
        self.moving_to_y = False
        self.moving_to_z = True
        self.moving_forward = False
        self.moving_backward = False
        self.intercepting_ball = [False, 0]
        self.strategy_4_data = {"forward": True, "function": 1}
        self.ball_intercept_pos = None
        self.ball_intercept_direction = 0
        self.initial_ball_pos = 0
        self.predicting = False
        self.arrived = False
        self.timer = 0
        self.stuck = False
        self.relocated = False
        self.relocation_pos = 0
        self.arrived_to_shoot = False
        self.ready_for_relocation = False
        self.last_ball_pos = [0, 0]
        self.real_speed = False
        self.mimic_coord = [0, 0]
        self.mimic_timer = 0
        self.mimic_stuck_pos = [0, 0]
        self.mimic_stuck = False
        self.mimic_flag = False
        self.stuck_timer = 0
        self.stuck_pos = [0, 0]

        self.flags = {
            "moving to x": False,
            "moving to y": False,
            "moving to z": True,
            "moving forward": False,
            "moving backward": False,
            "intercepting ball": [False, 0],
            "strategy 4 data": {
                "forward": True,
                "function": 1
            },
            "ball intercept pos": None,
            "ball intercept direction": 0,
            "initial ball pos": 0,
            "predicting": False,
            "arrived": False,
            "timer": 0,
            "stuck": False,
            "relocated": False,
            "relocation pos": 0,
            "arrived to shoot": False,
            "ready for relocation": False,
            "last ball pos": [0, 0],
            "real speed": False,
            "ball getting closer": False,
            "arrived at mimicPos": False,
            "going to mimicPos": False,
            "adjusted heading": False,
            "robot is stuck": False,
            "real ball speed": False,
            "real robot speed": False
        }

        self.start_time = time.time()

    def set_right_vel(self, v):
        self.right_motor.setVelocity(v)
        self.right_wheel_vel = v

    def set_left_vel(self, v):
        self.left_motor.setVelocity(v)
        self.left_wheel_vel = v

    def parse_supervisor_msg(self, packet: str) -> dict:
        """Parse message received from supervisor

        Returns:
            dict: Location info about each robot and the ball.
            Example:
                {
                    'waiting_for_kickoff': False,
                }
        """
        # True/False telling whether the goal was scored
        struct_fmt = "?"
        unpacked = struct.unpack(struct_fmt, packet)

        data = {"waiting_for_kickoff": unpacked[0]}
        return data

    def get_new_data(self) -> dict:
        """Read new data from supervisor

        Returns:
            dict: See `parse_supervisor_msg` method
        """
        packet = self.receiver.getData()
        self.receiver.nextPacket()

        return self.parse_supervisor_msg(packet)

    def is_new_data(self) -> bool:
        """Check if there is new data from supervisor to be received

        Returns:
            bool: Whether there is new data received from supervisor.
        """
        return self.receiver.getQueueLength() > 0

    def parse_team_msg(self, packet: str) -> dict:
        """Parse message received from team robot

        Returns:
            dict: Parsed message stored in dictionary.
        """
        struct_fmt = 'iffff?'
        unpacked = struct.unpack(struct_fmt, packet)

        data = {
            'robot_id': unpacked[0],
            'robot_pos': [unpacked[1], unpacked[2]],
            'ball_pos': [unpacked[3], unpacked[4]],
            'see the ball': unpacked[5]

        }

        if round(unpacked[3], 0) == -2:
            data["ball_pos"] = None

        return data

    def get_new_team_data(self) -> dict:
        """Read new data from team robot

        Returns:
            dict: See `parse_team_msg` method
        """
        packet = self.team_receiver.getData()
        self.team_receiver.nextPacket()
        return self.parse_team_msg(packet)

    def is_new_team_data(self) -> bool:
        """Check if there is new data from team robots to be received

        Returns:
            bool: Whether there is new data received from team robots.
        """
        return self.team_receiver.getQueueLength() > 0

    def send_data_to_team(self, robot_id, robot_pos, ball_pos, see_ball) -> None:
        """Send data to the team

        Args:
             robot_id (int): ID of the robot
        """
        struct_fmt = 'iffff?'
        data = [robot_id, *robot_pos, *ball_pos, see_ball]
        packet = struct.pack(struct_fmt, *data)
        self.team_emitter.send(packet)

    def get_new_ball_data(self) -> dict:
        """Read new data from IR sensor

        Returns:
            dict: Direction and strength of the ball signal
            Direction is normalized vector indicating the direction of the
            emitter with respect to the receiver's coordinate system.
            Example:
                {
                    'direction': [0.23, -0.10, 0.96],
                    'strength': 0.1
                }
        """
        _ = self.ball_receiver.getData()
        data = {
            "direction": self.ball_receiver.getEmitterDirection(),
            "strength": self.ball_receiver.getSignalStrength(),
        }
        self.ball_receiver.nextPacket()
        return data

    def is_new_ball_data(self) -> bool:
        """Check if there is new data from ball to be received

        Returns:
            bool: Whether there is new data received from ball.
        """
        return self.ball_receiver.getQueueLength() > 0

    def get_gps_coordinates(self) -> list:
        """Get new GPS coordinates

        Returns:
            List containing x and y values
        """
        gps_values = self.gps.getValues()
        return [gps_values[1], gps_values[0]]

    def get_compass_heading(self) -> float:
        """Get compass heading in radians

        Returns:
            float: Compass value in radians
        """
        compass_values = self.compass.getValues()

        # Add math.pi/2 (90) so that the heading 0 is facing opponent's goal
        rad = math.atan2(compass_values[0], compass_values[1]) + (math.pi / 2)
        if rad < -math.pi:
            rad = rad + (2 * math.pi)

        rad = rad * 180 / math.pi * -1

        if rad > 0:
            rad -= 180
        else:
            rad += 180

        return rad

    def get_sonar_values(self) -> dict:
        """Get new values from sonars.

        Returns:
            dict: Value for each sonar.
        """
        return {
            "left": self.sonar_left.getValue(),
            "right": self.sonar_right.getValue(),
            "front": self.sonar_front.getValue(),
            "back": self.sonar_back.getValue(),
        }

    def run(self):
        raise NotImplementedError
