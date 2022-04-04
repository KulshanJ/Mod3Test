from qset_lib import Rover
from time import sleep

import math
import numpy as np

def angleFinding(previousPosition, currentPosition):
    reference = np.array([0, 1])
    m = np.array(previousPosition)
    n = np.array(currentPosition)
    vector = np.subtract(n, m)
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
    if 0 < angle <= 90:
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
    heading = angleConvertion(rover.heading)
    desiredHeading = heading + angle
    if angle < 0:
        #turn right wheel backwards and left forwards
        left_side_speed = -1
        right_side_speed = 1
        rover.send_command(left_side_speed, right_side_speed)
    # if cw
    else:
        # turn right wheel forwards and left backwards
        left_side_speed = 1
        right_side_speed = -1
        rover.send_command(left_side_speed, right_side_speed)
    # if heading = new desired vector set wheels to 0

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
        listOfDifference[i] = listFromLiDAR[i] - listOfDistance1[i]
        i += 1

    listOfDifference.sort()
    '''
        Return false if the path is clear ---> keep moving straightforward
        return true if the obstacle is within its path ---> need further actions
    '''
    result = False
    if(listOfDifference[0] < 0):
        result = True
    closestDistance = listOfDifference[0]*(-1)
    return closestDistance, result


def getLiDARDistance():
    rover = Rover()
    listOfLiDAR = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                   100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
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

def distanceChecking2(listFromLiDAR, flag):

    deltaAngle = math.pi / (2 * 29)
    roverRange = math.sqrt(0.61 ** 2 + 0.61 ** 2)
    listOfAlertDistance2 = []
    i = 0
    while i < 30:
        listOfAlertDistance2.append(roverRange / math.sin((math.pi / 4) + (deltaAngle * (i))))
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
flag == 1 && true : something is in the path as well as the turning radius ---> stop and scan
flag == 1 && false: path is clear but the rover may not be able to turn ---> keep moving forward
flag > 1 && true: stuck ---> turn into Lucas's function, which is moving back and scaning
flag == 0 && true: something in the path ---> stop and scan
flag == 0 && false: all is clear ---> move forward
'''


def stop_check():
    rover = Rover()

    # variables for sleep time, speed, stopping distance, minimum tolerated distance
    speed = 1
    sleeptime = 3

    # Stop the rover first
    left_side_speed = 0
    right_side_speed = 0
    rover.send_command(left_side_speed, right_side_speed)
    closestDist = [0, 0, 0, 0]

    # call on turning function to rotate 90 degrees ccw
    turningFunction(-90)
    listFromLiDAR = getLiDARDistance()
    closestDistance0,result = distanceChecking1(listFromLiDAR)
    closestDist[0] = closestDistance0



    # call on turning rotate 45 cw
    turningFunction(45)
    listFromLiDAR = getLiDARDistance()
    closestDistance1, result = distanceChecking1(listFromLiDAR)
    closestDist[1] = closestDistance1


    # call on turning rotate 90 cw
    turningFunction(90)
    listFromLiDAR = getLiDARDistance()
    closestDistance2, result = distanceChecking1(listFromLiDAR)
    closestDist[2] = closestDistance2

    # call on turning rotate 45 cw
    turningFunction(45)
    listFromLiDAR = getLiDARDistance()
    closestDistance3, result = distanceChecking1(listFromLiDAR)
    closestDist[3] = closestDistance3

    # call on turning to rotate 90 ccw and return to original position
    turningFunction(-90)
    biggestIndex =closestDist.index(max(closestDist))

    if closestDist[biggestIndex] <= 3:
        backup1()
    else:
        if biggestIndex <= 1:
            turningFunction(-90+45*biggestIndex)
        else:
            turningFunction(-45+45*biggestIndex)

        left_side_speed = speed
        right_side_speed = speed
        rover.send_command(left_side_speed, right_side_speed)
        time.sleep(sleeptime)

    left_side_speed = 0
    right_side_speed = 0
    rover.send_command(left_side_speed, right_side_speed)



# BACKUP

def backup1():
    rover = Rover()
    previousX = rover.x
    previousY = rover.y
    left_side_speed = -1
    right_side_speed = -1

    while 1:
        rover.send_command(left_side_speed, right_side_speed)
        currentX = rover.x
        currentY = rover.y
        deltaDistance = (previousX-currentX)**2+(previousY-currentY)**2
        if deltaDistance >= 4:
            left_side_speed = 0
            right_side_speed = 0
            rover.send_command(left_side_speed, right_side_speed)
            break





def main():
    destinationList = [100, 100]
    rover = Rover()

    # Step1: orient the rover to head towards the destination
    '''Call function to get current position and head vector'''
    currentPosition = [rover.x, rover.y]

    '''Find angles and make orientation'''
    angleOfHeadingVector = angleFinding(currentPosition, destinationList)
    #heading = rover.heading
    #angleOfRoverHead = angleConvertion(heading)
    #angleToTurn = pathDecision(angleOfRoverHead, angleOfHeadingVector)
    #turningFunction(angleToTurn)
    # fix later
    # none of this works because heading isn't called


    # Step2: Move forward and scanning before bumping into obstacles
    i = 0
    left_side_speed = 5
    right_side_speed = 5

    while i < 3000:
        rover.send_command(left_side_speed, right_side_speed)

        flag = 0

        listOfLiDAR = getLiDARDistance()
        closestDistance, result = distanceChecking1(listOfLiDAR)
        flag = distanceChecking2(listOfLiDAR, flag)
        if result == True:
            left_side_speed = 0
            right_side_speed = 0
            rover.send_command(left_side_speed, right_side_speed)
            stop_check()

        '''elif flag >= 1 and result == True:
            left_side_speed = 0
            right_side_speed = 0
            rover.send_command(left_side_speed, right_side_speed)
            backup1()'''

        i = i + 1
        sleep(0.01)

        if rover.x == destinationList[0] and rover.y == destinationList[1]:
            break


if __name__ == "__main__":
    main()
