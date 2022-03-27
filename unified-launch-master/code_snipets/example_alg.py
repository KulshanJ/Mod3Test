from qset_lib import Rover
from time import sleep

def main():
    
    rover = Rover()
    
    i = 0

    left_side_speed = 2
    right_side_speed = -2

    while i < 1000:
        

        for dist in rover.laser_distances:
            if dist < 2:
                    left_side_speed = 2
                    right_side_speed = -2
                    print("X: " + str(rover.x) + " Y: " + str(rover.y) + " Heading: " + str(rover.heading))
                
        rover.send_command(left_side_speed, right_side_speed)
        i = i + 1
        sleep(1.01)


if __name__ == "__main__":
    main()
