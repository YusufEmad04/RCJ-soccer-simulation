# rcj_soccer_player controller - ROBOT B3

# Feel free to import built-in libraries
import math
from functions import *

# You can also import scripts that you put into the folder with controller
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils


class MyRobot3(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:

            # check if there is data from (supervisor receiver)
            if self.is_new_data():

                # receive and print data (team + supervisor)
                data = receive_data(self)

                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    # data from the ball receiver (ball receiver)
                    ball_data = receive_ball_data(self, data["heading"], data["robot position"])

                    # move towards the ball
                    move_to_point(self, data["robot position"], data["heading"], ball_data["ball position"])

                    # Send message to team robots and prints
                    self.send_data_to_team(self.player_id, data["robot position"], ball_data["ball position"], True)

                # robot can't see the ball
                else:

                    # do something
                    # robot moves to origin
                    move_to_point(self, data["robot position"], data["heading"], [0, 0])

                    self.send_data_to_team(self.player_id, data["robot position"], [-2, -2], False)

