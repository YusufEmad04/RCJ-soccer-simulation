# rcj_soccer_player controller - ROBOT B2

# Feel free to import built-in libraries
import math
import functions
import Funtions2
# You can also import scripts that you put into the folder with controller
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils


class MyRobot2(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:

            # check if there is data from (supervisor receiver)
            if self.is_new_data():
                team_data = []

                # Get data from compass
                heading = self.get_compass_heading()

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()

                # data from the supervisor (supervisor receiver)
                data = self.get_new_data()

                # check if there is data from (team receiver)
                # while loop to empty queue
                while self.is_new_team_data():
                    # data from the team receiver (team receiver)
                    team_data.append(self.get_new_team_data())
                    team_data.append(self.get_new_team_data())
                
                Funtions2.get_g_angle(robot_pos,heading,[0,0])

                # check if there is data from (ball receiver)
                if self.is_new_ball_data():

                    left_speed = 0
                    right_speed = 0

                    # data from the ball receiver (ball receiver)
                    ball_data = self.get_new_ball_data()

                    robot_ball_angle = functions.get_angle(ball_data["direction"])
                    ball_distance = functions.get_ball_distance(ball_data["strength"])
                    ball_pos = functions.get_ball_position(heading, ball_distance, robot_ball_angle, robot_pos)
                    # if -20 <= robot_ball_angle <= 20:
                    #     left_speed = -5
                    #     right_speed = -5
                    # elif 20 < robot_ball_angle <= 180:
                    #     left_speed = -4
                    #     right_speed = 4
                    # elif -20 > robot_ball_angle >= -180:
                    #     left_speed = 4
                    #     right_speed = -4
                    #
                    # # Set the speed to motors
                    # self.left_motor.setVelocity(left_speed)
                    # self.right_motor.setVelocity(right_speed)

                    # Send message to team robots and prints
                    self.send_data_to_team(self.player_id, robot_pos, ball_pos, True)

                # robot can't see the ball
                else:

                    # do something
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)

                    self.send_data_to_team(self.player_id, robot_pos, [-2, -2], False)

