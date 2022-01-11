# rcj_soccer_player controller - ROBOT B3
# Feel free to import built-in libraries
# You can also import scripts that you put into the folder with controller

from functions import *
from functions2 import *
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        distance = 0
        ball_speedL = [0, 0, 0, 0]
        while self.robot.step(TIME_STEP) != -1:
            # each two loops time step is increased by 1
            increment_step(self)

            # check if there is data from (supervisor receiver)
            if self.is_new_data():

                # receive and print data (team + supervisor)
                data = receive_data(self)

                # get robot speed
                robot_speed = get_robot_speed(self)

                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    # data from the ball receiver (ball receiver)
                    ball_data = receive_ball_data(self, data["heading"], data["robot position"])

                    # get ball speed
                    ball_speed = get_ball_speed(self, True)
                    add_to_arr(ball_speedL,ball_speed[0])

                    # move towards the ball both when far and close
                    dist = get_dist(data["robot position"], ball_data["ball position"])
                    # distance to be updated when ball hit
                    """if dist < 0.3:
                        # Movement varies with distance
                        if abs(round(ball_speedL[-1], 1) - round(ball_speedL[-4], 1)) > 0.1:
                            distance = get_dist(data["robot position"], ball_data["ball position"])
                        move_Fwd(self, data["robot position"], data["heading"], ball_data["ball position"], distance)
                        print(round(ball_speed[0],1), "is the ball speed")
                        print(robot_speed, "is the robot speed")

                    else:
                        # Movement to ball when far
                        move_to_point(self, data["robot position"], data["heading"], ball_data["ball position"])"""
                    # Robot to match ball speed

                    # move_Fwd(self, data["robot position"], data["heading"], ball_data["ball position"], dist , ball_speed_new)
                    dir = 180
                    move_dir(self, data["robot position"], data["heading"], ball_data["ball position"], dist,
                             ball_speed, dir)

                    # Send message to team robots and prints
                    self.send_data_to_team(self.player_id, data["robot position"], ball_data["ball position"], True)

                # robot can't see the ball
                else:

                    # get ball speed from other robots
                    ball_speed = get_ball_speed(self, False, data["team data"])

                    # robot moves to origin
                    move_to_point(self, data["robot position"], data["heading"], [0, 0])

                    self.send_data_to_team(self.player_id, data["robot position"], [-2, -2], False)