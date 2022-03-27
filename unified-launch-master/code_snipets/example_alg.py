from qset_lib import Rover
from time import sleep


def main():
    
    rover = Rover()
    
    i = 0
    
    heading = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                      pose.orientation.w])[2] / math.pi * 180.0

    
    #heading = rover.heading

    left_side_speed = 7
    right_side_speed = -7

    while i < 1000:
        #print("X: " + rover.x + " Y: " + rover.y + " Heading: " + rover.heading)

        for dist in rover.laser_distances:
            if dist < 2:
                    #left_side_speed = -7
                    #right_side_speed = -7
                    print(heading)
                    

                
        rover.send_command(left_side_speed, right_side_speed)
        i = i + 1
        sleep(0.01)


if __name__ == "__main__":
    main()
