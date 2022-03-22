# rcj_soccer_player controller - ROBOT B3
# Feel free to import built-in libraries
# You can also import scripts that you put into the folder with controller
import time

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
                # assign_role(self)
                check_strategy(self)
                print("robot 1: {}".format(self.roles[0]))

                # get_real_ball_status(self)
                # if self.ball_status in [1, 4]:
                #     if abs(self.ball_pos_arr[-1][1]) > 0.15 and self.ball_pos_arr[-1][0] > 0.4:
                #         print("corner")
                #         go_to_corner2(self)
                #     else:
                #         print("defend")
                #         defend_mimic_at_goal(self)
                # else:
                #     self.set_left_vel(0)
                #     self.set_right_vel(0)
                # adjust_robot_penalty_ti me(self)
                # print("robot in penalty area: {}".format(self.flags["robot in penalty area"]))
                # print("outside timer: {}".format(time.time() - self.outside_timer))
                # print("penalty area timer: {}".format(time.time() - self.penalty_area_timer))
                # print("--------------------")


                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    # data from the ball receiver (ball receiver)
                    ball_data = receive_ball_data(self)
                    
                    ball_speed = get_ball_speed(self)
                    # if 30 >= ball_speed[1] >= -30:
                    #     print("Avoiding")
                    #     avoid_object(self)
                    # else:
                    #     print("BALL")
                    #     move_to_point(self, ball_speed[2])
                    # ready_to_defend(self)
                    
                    # defend(self)
                    # intercept_ball(self)

                    # Send message to team robots and prints
                    send_team_data(self)

                # robot can't see the ball
                else:

                    get_team_ball_data(self)

                    # defend(self)
                    # intercept_ball(self)

                    send_team_data(self, False)
