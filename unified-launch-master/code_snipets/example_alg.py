from qset_lib import Rover
from time import sleep




rover = Rover()

i = 0

left_side_speed = -7
right_side_speed = 7

rover.send_command(left_side_speed, right_side_speed)


