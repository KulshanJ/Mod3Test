from qset_lib import Rover
from time import sleep

def main():
    
    rover = Rover()
    
    i = 0

    left_side_speed = 7
    right_side_speed = 7
    
    listOfLiDAR = []

    for dist in rover.laser_distances:
        listOfLiDAR.append(dist)

    while i < 1000:
        #print("X: " + str(rover.x) + " Y: " + str(rover.y) + " Heading: " + str(rover.heading))
        
        for dist in rover.laser_distances:
            listOfLiDAR.append(dist)
        print(listOfLiDAR)         
                
        rover.send_command(left_side_speed, right_side_speed)
        i = i + 1
        sleep(0.5)


if __name__ == "__main__":
    main()
