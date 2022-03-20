# rcj_soccer_player controller - ROBOT B3
# Feel free to import built-in libraries
# You can also import scripts that you put into the folder with controller
import time

from functions import *
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot3(RCJSoccerRobot):
    def run(self):

        while self.robot.step(TIME_STEP) != -1:
            # each two loops time step is increased by 1
            increment_step(self)

            # check if there is data from (supervisor receiver)
            if self.is_new_data():

                # receive and print data (team + supervisor)
                data = receive_data(self)
                # print("id: {}, {}".format(self.player_id,self.team_data))
                # check_strategy(self)
                # print("robot 3: {}".format(self.roles[2]))
                # self.set_left_vel(7.3)
                # self.set_right_vel(7.3)
                # print("b3: {}".format(self.roles[2]))

                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    # data from the ball receiver (ball receiver)
                    ball_data = receive_ball_data(self)
                    # print("speed: {}, dir: {}".format(*get_ball_speed(self)[:2]))
                    #
                    # if self.flags["intercepting ball"][0]:
                    #     defend_strategy_2(self)
                    # else:
                    #     defend_strategy_2(self, False)


                    # move_to_point(self, ball_data["ball position"])

                    # Send message to team robots and prints
                    send_team_data(self)

                # robot can't see the ball
                else:

                    get_team_ball_data(self)

                    # robot moves to origin
                    # move_to_point(self, [0, 0])

                    send_team_data(self, False)
