# rcj_soccer_player controller - ROBOT B3
# Feel free to import built-in libraries
# You can also import scripts that you put into the folder with controller

from functions import *
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            # each two loops time step is increased by 1
            increment_step(self)

            # check if there is data from (supervisor receiver)
            if self.is_new_data():

                # receive and print data (team + supervisor)
                data = receive_data(self)

                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    # data from the ball receiver (ball receiver)
                    ball_data = receive_ball_data(self)
                    defend(self)

                    # move towards the ball
                    # move_to_point(self, data["robot position"], data["heading"], ball_data["ball position"], True)

                    # Send message to team robots and prints
                    self.send_data_to_team(self.player_id, self.robot_pos_arr[-1], self.ball_pos_arr[-1], True)

                # robot can't see the ball
                else:

                    get_team_ball_data(self)
                    defend(self)

                    # robot moves to origin
                    # move_to_point(self, data["robot position"], data["heading"], [0, 0], True)

                    self.send_data_to_team(self.player_id, data["robot position"], [-2, -2], False)
