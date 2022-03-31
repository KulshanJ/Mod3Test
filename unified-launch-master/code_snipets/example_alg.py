from qset_lib import Rover
from time import sleep

import math
import numpy as np


def angleFindingByTwoLists(previousPosition, currentPosition):
    '''
    Previous ---> tail of the vector
    Current ---> head of the vector
    '''

    '''
    Findingh the heading vector
    '''
    m = np.array(previousPosition)
    n = np.array(currentPosition)
    vector = np.subtract(n, m)
    return angleFindingByVector(vector)


def angleFindingByVector(vector):
    reference = np.array([0, 1])

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
    checkingVector = np.array([1, 0])
    check = np.dot(checkingVector, vector)

    '''
    Convert the small angle in 2/3 quadrant to the big angle we want
    '''
    if check < 0:
        angle = 2 * math.pi - angle
    return math.degrees(angle)



def angleConvertion(angle):
    if 0 < angle <= 90 :
        angle = 90-angle
    elif 90 < angle <= 180:
        angle = 360 - (angle - 90)
    elif -180 < angle <= 0:
        angle = 90 - angle
    return angle



# heading
def turningFunction(angle):
    # if ccw
    rover = Rover()
    if angle < 0:
        
        #turn right wheel backwards and left forwards
        left_side_speed = -3
        right_side_speed = 3
        rover.send_command(left_side_speed, right_side_speed)
    # if cw
    else:
       
        # turn right wheel forwards and left backwards
        left_side_speed = 3
        right_side_speed = -3
        rover.send_command(left_side_speed, right_side_speed)
    # if heading = new desired vector set wheels to 0
    heading = angleConvertion(heading)
    desiredHeading = heading + angle
    x = desiredHeading - 10
    y = desiredHeading + 10

    check = 1
    # heading < y and x < heading
    while check == 1:
        realheading = angleConvertion(rover.heading)
        if realheading == desiredHeading:
            left_side_speed = 0
            right_side_speed = 0
            rover.send_command(left_side_speed, right_side_speed)
            check = 0


def pathDecision(currentAngle, desiredAngle):
    angleToTurn = desiredAngle - currentAngle
    return angleToTurn


def distanceChecking1(listFromLiDAR):
    '''
    Calculate the delta distance and will return a list containing the difference
    '''
    roverWidth = 0.61
    listOfDistance1 = []
    i = 0
    deltaAngle = math.pi / (2 * 29)
    while i < 30:
        listOfDistance1.append(roverWidth / math.sin((math.pi / 4) + (deltaAngle * (i))))
        i += 1
        
    listOfDifference = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    i = 0
    while i < 30:
        listOfDifference[i] = listFromLiDAR[i] - listOfAlertDistance1[i]
        i += 1

    listOfDifference.sort()
    '''
        Return false if the path is clear ---> keep moving straightforward
        return true if the obstacle is within its path ---> need further actions
    '''
    return listOfDifference[0]*(-1), any(n < 0 for n in listOfDifference)

def getLiDARDistance():
    rover = Rover()
    listOfLiDAR = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                       100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
    k = 0
    for dist in rover.laser_distances:
        if dist < 100:
                listOfLiDAR[k] = dist
            
        k = k + 1
    return listOfLiDAR
'''
flag will be defined in main function
the value will be set to default(0) after every stop and scan
'''
'''
def distanceChecking2(listOfAlertDistance2, listFromLiDAR, flag):
    deltaAngle = math.pi / (2 * 29)
    
    roverRange = math.sqrt(0.61 ** 2 + 0.61 ** 2)
    listOfDistance2 = []
    i = 0
    while i < 30:
        listOfDistance2.append(roverRange / math.sin((math.pi / 4) + (deltaAngle * (i))))
        i += 1

    listOfDifference = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    i = 0
    while i < 30:
        listOfDifference[i]=listFromLiDAR[i] - listOfAlertDistance2[i]
        i += 1

    if any(n < 0 for n in listOfDifference) == True:

        flag = flag + 1
    else:
        flag = 0

    return flag
'''
'''
flag == 1 && true : something is in the path as well as the turning radius ---> stop and scan
flag == 1 && false: path is clear but the rover may not be able to turn ---> keep moving forward
flag > 1 && true: stuck ---> turn into Lucas's function, which is moving back and scaning
flag == 0 && true: something in the path ---> stop and scan
flag == 0 && false: all is clear ---> move forward
'''


def stop_check():
    rover = Rover()
    # variables for sleep time, speed, stopping distance, minimum tolerated distance
    speed = 3
    sleeptime = 3
    closestdist = 2
    mindist = 3

    # call on lidar function to determine distance from object
    
    left_side_speed = 0
    right_side_speed = 0
    rover.send_command(left_side_speed, right_side_speed)

    # call on turning function to rotate 45 degrees ccw
    turningFunction(-45)
    # call on lidar function to determine distance from object
    listFromLiDAR = getLiDARDistance()
    closestDistance,result = distanceChecking1(listFromLiDAR)
    
  
    if 0<closestDistance < 10:
            ccw45dist = closestDistance
    else:
            ccw45dist = 10
    # set this distance to be the greatest distance
    greatestdist = ccw45dist

    # call on turning rotate 45 ccw
    turningFunction(-45)
    listFromLiDAR = getLiDARDistance()
    closestDistance, result = distanceChecking1(listFromLiDAR)
    
    if 0< closestDistance < 10:
            ccw90dist = closestDistance
    else:
            ccw90dist = 10
        
    if ccw90dist > greatestdist:
            greatestdist = ccw90dist
    
    # call on turning rotate 180 cw
    turningFunction(180)
    listFromLiDAR = getLiDARDistance()
    closestDistance, result = distanceChecking1(listFromLiDAR)
    if 0< closestDistance < 10:
            cw90dist = closestDistance
    else:
            cw90dist = 10
    if cw90dist > greatestdist:
            greatestdist = cw90dist

    # call on turning rotate 45 ccw
    turningFunction(-45)
    listFromLiDAR = getLiDARDistance()
    closestDistance, result = distanceChecking1(listFromLiDAR)
    
    if 0< closestDistance < 10:
            cw45dist = closestDistance
    else:
            cw45dist = 10
    
    if cw45dist > greatestdist:
            greatestdist = cw45dist

        # call on turning to rotate 45 ccw and return to original position
    turningFunction(-45)
       # now choose which distance to go. If 45 ccw is greater than 5 meters go this way automatically for sleeptime seconds
        # if not, but 45 cw is greater than 5 meters, go this way for sleeptime seconds. If neither of these are true,
        # pick the heading with the greatest distance and travel in that direction. If all distances are less than
        # mindist, call on Lucas's function
    
    if ccw45dist > 5:
                # rotate 45 ccw
                turningFunction(-45)
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)
                
    elif cw45dist > 5:
                # rotate 45 cw
                turningFunction(45)
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

            # if neither are true, pick heading with greatest distance
    elif greatestdist == cw45dist:
                # rotate 45 cw
                turningFunction(45)
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

    elif greatestdist == ccw45dist:
                # rotate 45 ccw
                turningFunction(-45)
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

    elif greatestdist == cw90dist:
                # rotate 90 cw
                turningFunction(90)
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)

    elif greatestdist == ccw90dist:
                # rotate 90 ccw
                turningFunction(-90, heading)
                left_side_speed = speed
                right_side_speed = speed
                rover.send_command(left_side_speed, right_side_speed)
                time.sleep(sleeptime)


# BACKUP

# reversing
def backup1():
    rover = Rover()

    i = -5
    j = -5
    left_side_speed = i
    right_side_speed = j

    rover.send_command(left_side_speed, right_side_speed)

    # call Eden's function
    if __name__ == "__stop_check__":
        stop_check()





# turning
def backup2():
    rover = Rover()

    while 1:

        i = 5
        j = -5

        left_side_speed = i
        right_side_speed = j

        rover.send_command(left_side_speed, right_side_speed)

        # rotate 45 ccw, check if initial/final heading vectors equal

        if initialHeadingVector == finalHeadingVector:
            if __name__ == "__stop_check__":
                stop_check()

        i = -5
        j = 5

        left_side_speed = i
        right_side_speed = j

        rover.send_command(left_side_speed, right_side_speed)

        # rotate 45 cw, check if initial/final heading vectors equal
        if initialHeadingVector == finalHeadingVector:
            if __name__ == "__stop_check__":
                stop_check()

    while 1:
        sleeptime = 5

        left_side_speed = 5
        right_side_speed = 5

        time.sleep(sleeptime)

        rover.send_command(left_side_speed, right_side_speed)

        # call in Sean's function

    # reorient, use Sean's function

    # call Eden's function

    if __name__ == "__stop_check__":
        stop_check()


destinationList = [100, 100]


def main():
    rover = Rover()

    '''
    Calculating the alert distance for each of the 32 'lines'
    Set the list as global
    '''
    
    

    # Step1: orient the rover to head towards the destination
    '''Call function to get current position and head vector'''
    currentPosition = [rover.x, rover.y]
    headVector = [math.sin(rover.heading), math.cos(rover.heading)]

    '''Find angles and make orientation'''
    angleOfHeadingVector = angleFindingByTwoLists(currentPosition, destinationList)
    angleOfRoverHead = angleFindingByVector(headVector)
    angleToTurn = pathDecision(angleOfRoverHead, angleOfHeadingVector)

    turningFunction(angleToTurn)
    # fix later
    # none of this works because heading isnt called


    # Step2: Move forward and scanning before bumping into obstacles
    i = 0
    left_side_speed = 5
    right_side_speed = 5

    while i < 3000:
        rover.send_command(left_side_speed, right_side_speed)

        # flag = 0
        heading = rover.heading
        
        listOfLiDAR = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                       100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
        k = 0
        for dist in rover.laser_distances:
            if dist < 100:
                listOfLiDAR[k] = dist
                k = k + 1

        closestDistance, result = distanceChecking1(listOfLiDAR)
        # flag = distanceChecking2(listOfDistance2, listOfLiDAR, flag)
        if result == True:
            stop_check()
            
            # elif flag > 1:
            # Call Lucas's function

            i = i + 1
        sleep(0.01)
        
        if rover.x == destinationList[0] and rover.y == destinationList[1]:
            break


if __name__ == "__main__":
    main()
