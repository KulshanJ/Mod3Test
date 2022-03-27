from qset_lib import Rover
from time import sleep

import math
import numpy as np


def angleFinding(previousPosition, currentPosition):
    '''
    Previous ---> tail of the vector
    Current ---> head of the vector
    '''
    reference = np.array([0,1])

    '''
    Finding the heading vector
    '''
    m = np.array(previousPosition)
    n = np.array(currentPosition)
    vector = np.subtract(n,m)


    '''
    According to vector dot product rule, calculate the angle between the vector and the reference vector
    '''
    products = np.dot(vector, reference)
    cosX = products / (np.linalg.norm(vector))
    angle = math.acos(cosX)

    '''
    We want to track the angle from the +y direction to the vector, 
    but the angle returned is always the smaller one
    So we check here to see if the vector is in 1/4 quadrant or 2/3 quadrant
    '''
    checkingVector = np.array([1,0])
    check = np.dot(checkingVector,vector)

    '''
    Convert the small angle in 2/3 quadrant to the big angle we want
    '''
    if check < 0:
        angle= 2* math.pi - angle

    return angle



def pathDecision(currentAngle, desiredAngle):
    angleToTurn = desiredAngle - currentAngle
    return angleToTurn




def turning (angleToTurn):
    if angleToTurn < 0:
        '''Turning counter clockwise here'''
    else:
        '''Turning clockwise here'''



'''
Calculating the alert distance for each of the 32 'lines'
Set the list as global
'''
roverWidth = 0.61
listOfDistance1 = [roverWidth / math.sin(math.pi/4)]
i = 2
deltaAngle = math.pi/(2*31)
while i <= 32:
    listOfDistance1.append(roverWidth / math.sin((math.pi/4) + (deltaAngle*(i-1))))
    i += 1



roverRange = math.sqrt(0.61**2 + 0.61 ** 2)
listOfDistance2 = [roverRange / math.sin(math.pi/4)]
i = 2
while i <= 32:
    listOfDistance2.append(roverRange / math.sin((math.pi/4) + (deltaAngle*(i-1))))
    i += 1





def distanceChecking1(listOfAlertDistance1, listFromLiDAR):
    '''
    Calculate the delta distance and will return a list containing the difference
    '''

    listOfDifference = np.array(np.array(listFromLiDAR) - np.array(listOfAlertDistance1)).tolist()

    '''
        Return false if the path is clear ---> keep moving straightforward
        return true if the obstacle is within its path ---> need further actions
    '''
    return listOfDifference, any(n < 0 for n in listOfDifference)

'''
flag will be defined in main function
the value will be set to default(0) after every stop and scan
'''
def distanceChecking2(listOfAlertDistance2, listFromLiDAR, flag):
    '''
    Calculate the delta distance and will return a list containing the difference
    '''
    listOfDifference = np.array(np.array(listFromLiDAR) - np.array(listOfAlertDistance2)).tolist()

    if any(n < 0 for n in listOfDifference) == True:
        '''means something is in the turning radius'''
        flag = flag + 1
    else:
        flag = 0
    '''
    If the flag is 1, indicating something is within the turning radius of the rover.
    
    If the flag is over 0, means the rover has moved forward under the condition that there's not enough space for 
    turning. In this case, the rover may already be stuck.
    
    
    '''
    return flag

'''
flag == 1 && true : something is in the path as well as the turning radius ---> stop and scan
flag == 1 && false: path is clear but the rover may not be able to turn ---> keep moving forward
flag > 1 && true: stuck ---> turn into Lucas's function, which is moving back and scaning
flag == 0 && true: something in the path ---> stop and scan
flag == 0 && false: all is clear ---> move forward
'''






def stop_check():

    # variables for sleep time, speed, stopping distance, minimum tolerated distance
    speed = 1
    sleeptime = 2
    closestdist = 0.5
    mindist = 2

    # call on lidar function to determine distance from object
    objectdist = 0.5
    if objectdist <= closestdist:
        left_side_speed = 0
        right_side_speed = 0
        rover.send_command(left_side_speed, right_side_speed)

        # call on turning function to rotate 45 degrees ccw
        # call on lidar function to determine distance from object
        ccw45dist = 3

        # set this distance to be the greatest distance
        greatestdist = ccw45dist

        # call on turning rotate 45 ccw
        ccw90dist = 4
        if ccw90dist > greatestdist:
            greatestdist = ccw90dist

        # call on turning rotate 180 cw
        cw90dist = 6
        if cw90dist > greatestdist:
            greatestdist = cw90dist

        # call on turning rotate 45 ccw
        cw45dist = 6
        if cw45dist > greatestdist:
            greatestdist = cw45dist

        # call on turning to rotate 45 ccw and return to original position
        # now choose which distance to go. If 45 ccw is greater than 5 meters go this way automatically for sleeptime seconds
        # if not, but 45 cw is greater than 5 meters, go this way for sleeptime seconds. If neither of these are true,
        # pick the heading with the greatest distance and travel in that direction. If all distances are less than
        # mindist, call on Lucas's function

        if cw45dist <= mindist & cw90dist <= mindist & ccw90dist <= mindist & ccw45dist <= mindist:
            # call on Lucas's function

        else:
            if ccw45dist > 5:
                # rotate 45 ccw
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)
            elif cw45dist > 5:
                # rotate 45 cw
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

            # if neither are true, pick heading with greatest distance
            if greatestdist == cw45dist:
                # rotate 45 cw
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

            elif greatestdist == ccw45dist:
                # rotate 45 ccw
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

            elif greatestdist == cw90dist:
                # rotate 90 cw
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

            elif greatestdist == ccw90dist:
                # rotate 90 ccw
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)













# Lidar Section. Outputs lift of x and y coordinates as well as how any obstacles are detected.
def Lidar_Coordinate_Code():
        X = 1
        Counter = 0
        for dist in rover.laser_distances:

            # Find the theta and laser distance using geometry and the laser call function
            lasertheta =  (pi/2) - ((X - 1)((pi)/32))
            laserdist = dist

            if laserdist < 100:
                # Calculate the x and y only if the sensor reads something. Count how many data points there are
                 ycoord[X] = laserdist*math.sin(lasertheta)
                 xcoord[X] = laserdist*math.cos(lasertheta)
                 X = X + 1

        return ycoord, xcoord, X

    
def main():
    
    rover = Rover()
    
    #Step1: orient the rover to head towards the destination
    '''Call function to get current position and head vector'''
    currentPosition = [rover.x, rover.y]
    headVector = 

    
    '''Find angles and make orientation'''
    angleOfHeadingVector = angleFindingByTwoLists(currentPosition, destinationList)
    angleOfRoverHead = angleOfHeadingVector(headVector)
    turning(pathDecision(angleOfRoverHead, angleOfHeadingVector))

 
    #Step2: Move forward and scanning before bumping into obstacles
   
    left_side_speed = 1
    right_side_speed = 1

    while i < 3000:
        rover.send_command(left_side_speed, right_side_speed)
        
        listOfDistance, result = distanceChecking1(listOfAlertDistance1, rover.laser_distance) 
        if result == true:
            stop_check()
        
            
        
        i = i + 1
        sleep(0.01)


if __name__ == "__main__":
    main()
