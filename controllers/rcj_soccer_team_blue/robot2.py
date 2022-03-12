# rcj_soccer_player controller - ROBOT B3
# Feel free to import built-in libraries
# You can also import scripts that you put into the folder with controller

from functions import *
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def run(self):

        while self.robot.step(TIME_STEP) != -1:
            # each two loops time step is increased by 1
            increment_step(self)

            # check if there is data from (supervisor receiver)
            if self.is_new_data():

                # receive and print data (team + supervisor)
                data = receive_data(self)
                # assign_role(self)
                # if self.roles[1] == 1:
                #     if self.flags["intercepting ball"][0]:
                #         defend_strategy_2(self)
                #     else:
                #         defend_strategy_2(self, False)
                # elif self.roles[1] == 2:
                #     defend_mimic_at_goal(self)
                # elif self.roles[1] == 4:
                #     if self.ball_pos_arr:
                #         move_to_point(self, self.ball_pos_arr[-1])
                # else:
                #     self.set_left_vel(0)
                #     self.set_right_vel(0)
                # print("robot 2: {}".format(self.roles[1]))

                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    # data from the ball receiver (ball receiver)
                    ball_data = receive_ball_data(self)

                    # move towards the ball
                    # move_to_point(self, ball_data["ball position"])

                    # Send message to team robots and prints
                    send_team_data(self)

                # robot can't see the ball
                else:

                    get_team_ball_data(self)

                    # robot moves to origin
                    # move_to_point(self, [0, 0])

                    send_team_data(self, False)
