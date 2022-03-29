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






def turningFunction(angle):
    # if cw

    rover = Rover()
    if angle < 0:
        # calculate the new desired vector
        heading = rover.heading
        heading = angleConvertion(heading)
        
        desiredHeading = heading + angle
        # turn right wheel backwards and left forwards
        left_side_speed = 1
        right_side_speed = -1
        rover.send_command(left_side_speed, right_side_speed)
    # if ccw
    else:
        # calculate the new desired vector
        heading = rover.heading
        heading = angleConvertion(heading)
        
        desiredHeading = heading + angle
        # turn right wheel forwards and left backwards
        left_side_speed = -1
        right_side_speed = 1
        rover.send_command(left_side_speed, right_side_speed)
    # if heading = new desired vector set wheels to 0
    if heading == desiredHeading:
        left_side_speed = 0
        right_side_speed = 0
        rover.send_command(left_side_speed, right_side_speed)






def pathDecision(currentAngle, desiredAngle):
    angleToTurn = desiredAngle - currentAngle
    return angleToTurn



def distanceChecking1(listOfAlertDistance1, listFromLiDAR):
    '''
    Calculate the delta distance and will return a list containing the difference
    '''

    listOfDifference = [] 
    i = 0
    while i < 30:
      listOfDifference.append(listFromLiDAR[i] - listOfAlertDistance1[i])
      i += 1
    listOfDifference.sort()
    '''
        Return false if the path is clear ---> keep moving straightforward
        return true if the obstacle is within its path ---> need further actions
    '''
    return listOfDifference[0], any(n < 0 for n in listOfDifference)

'''
flag will be defined in main function
the value will be set to default(0) after every stop and scan
'''
def distanceChecking2(listOfAlertDistance2, listFromLiDAR, flag):
    '''
    Calculate the delta distance and will return a list containing the difference
    '''
    listOfDifference = [] 
    i = 0
    while i < 30:
        listOfDifference.append(listFromLiDAR[i] - listOfAlertDistance2[i])
        i += 1
  
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
    objectdist = closestdistance #not to be confused with closestdist... this calls on Sean's function
    if objectdist <= closestdist:
            left_side_speed = 0
            right_side_speed = 0
            rover.send_command(left_side_speed, right_side_speed)

            # call on turning function to rotate 45 degrees ccw
            turningfunction(-45)
            # call on lidar function to determine distance from object
            if closestdistance < 10:
                ccw45dist = closestdistance
            else:
                ccw45dist = 10

            # set this distance to be the greatest distance
            greatestdist = ccw45dist

            # call on turning rotate 45 ccw
            turningfunction(-45)
            if closestdistance < 10:
                ccw90dist = closestdistance
            else:
                ccw90dist = 10
            if  ccw90dist > greatestdist:
                greatestdist = ccw90dist

            # call on turning rotate 180 cw
            turningfunction(180)
            if closestdistance < 10:
                ccw90dist = closestdistance
            else:
                ccw90dist = 10
            if cw90dist > greatestdist:
                greatestdist = cw90dist

            # call on turning rotate 45 ccw
            turningfunction(-45)
            if closestdistance < 10:
                cw45dist = closestdistance
            else:
                cw45dist = 10
            if cw45dist > greatestdist:
                greatestdist = cw45dist

            # call on turning to rotate 45 ccw and return to original position
            turningfunction(-45)
            # now choose which distance to go. If 45 ccw is greater than 5 meters go this way automatically for sleeptime seconds
            # if not, but 45 cw is greater than 5 meters, go this way for sleeptime seconds. If neither of these are true,
            # pick the heading with the greatest distance and travel in that direction. If all distances are less than
            # mindist, call on Lucas's function

            if cw45dist <= mindist & cw90dist <= mindist & ccw90dist <= mindist & ccw45dist <= mindist:
                # call on Lucas's function
                backup1()

            else:
                if ccw45dist > 5:
                    # rotate 45 ccw
                    turningfunction(-45)
                    left_side_speed = speed
                    right_side_speed = speed
                    rover.send_command(left_side_speed, right_side_speed)
                    time.sleep(sleeptime)
                elif cw45dist > 5:
                    # rotate 45 cw
                    turningfunction(45)
                    left_side_speed = speed
                    right_side_speed = speed
                    rover.send_command(left_side_speed, right_side_speed)
                    time.sleep(sleeptime)

                # if neither are true, pick heading with greatest distance
                if greatestdist == cw45dist:
                    # rotate 45 cw
                    turningfunction(45)
                    left_side_speed = speed
                    right_side_speed = speed
                    rover.send_command(left_side_speed, right_side_speed)
                    time.sleep(sleeptime)

                elif greatestdist == ccw45dist:
                    # rotate 45 ccw
                    turningfunction(-45)
                    left_side_speed = speed
                    right_side_speed = speed
                    rover.send_command(left_side_speed, right_side_speed)
                    time.sleep(sleeptime)

                elif greatestdist == cw90dist:
                    # rotate 90 cw
                    turningfunction(90)
                    left_side_speed = speed
                    right_side_speed = speed
                    rover.send_command(left_side_speed, right_side_speed)
                    time.sleep(sleeptime)

                elif greatestdist == ccw90dist:
                    # rotate 90 ccw
                    turningfunction(-90)
                    left_side_speed = speed
                    right_side_speed = speed
                    rover.send_command(left_side_speed, right_side_speed)
                    time.sleep(sleeptime)




#BACKUP

#reversing
def backup1(inputDistance) :

    i = -1
    j = -1
    left_side_speed = i
    right_side_speed = j

    rover.send_command(left_side_speed, right_side_speed)

    #call Eden's function
    if __name__ == "__stop_check__":
        stop_check()

initialHeadingVector = 0

#turning
def backup2(inputDistance) :

    while 1:

        i = 1
        j = -1

        left_side_speed = i
        right_side_speed = j

        rover.send_command(left_side_speed, right_side_speed)

        #rotate 45 ccw, check if initial/final heading vectors equal

        if initialHeadingVector == finalHeadingVector :
            break

        i = -1
        j = 1

        left_side_speed = i
        right_side_speed = j

        rover.send_command(left_side_speed, right_side_speed)

        # rotate 45 cw, check if initial/final heading vectors equal
        if initialHeadingVector == finalHeadingVector:
            break

    while 1 :

        sleeptime = 5

        left_side_speed = 1
        right_side_speed = 1

        time.sleep(sleeptime)

        rover.send_command(left_side_speed, right_side_speed)

        #call in Sean's function


    #reorient, use Sean's function


    #call Eden's function

    if __name__ == "__stop_check__":
        stop_check()    
    
destinationList = [100, 100]
    
    
def main():
    
    rover = Rover()
    
    
    
    '''
    Calculating the alert distance for each of the 32 'lines'
    Set the list as global
    '''
    roverWidth = 0.61
    listOfDistance1 = []
    i = 0
    deltaAngle = math.pi/(2*29)
    while i < 30:
      listOfDistance1.append(roverWidth / math.sin((math.pi/4) + (deltaAngle*(i))))
      i += 1



    roverRange = math.sqrt(0.61**2 + 0.61 ** 2)
    listOfDistance2 = []
    i = 0
    while i < 30:
        listOfDistance2.append(roverRange / math.sin((math.pi/4) + (deltaAngle*(i))))
        i += 1
    
    #Step1: orient the rover to head towards the destination
    '''Call function to get current position and head vector'''
    currentPosition = [rover.x, rover.y]
    headVector = [math.sin(rover.heading), math.cos(rover.heading)]

    
    '''Find angles and make orientation'''
    angleOfHeadingVector = angleFindingByTwoLists(currentPosition, destinationList)
    angleOfRoverHead = angleFindingByVector(headVector)
    turningFunction(pathDecision(angleOfRoverHead, angleOfHeadingVector))

    i = 0
 
    #Step2: Move forward and scanning before bumping into obstacles
   
    left_side_speed = 1
    right_side_speed = 1

    while i < 3000:
        rover.send_command(left_side_speed, right_side_speed)
        
        flag = 0
        
        listOfLiDAR = []

        for dist in rover.laser_distances:
            if dist<100:
                listOfLiDAR.append(dist)
            else:
                listOfLiDAR.append(100)
           
        closestDistance, result = distanceChecking1(listOfDistance1, listOfLiDAR ) 
        flag = distanceChecking2(listOfDistance2, listOfLiDAR, flag)
        if (flag == 1 or flag == 0 ) and result == True:
            stop_check()
        elif flag > 1:
            #Call Lucas's function
        
        
            i = i + 1
        sleep(0.01)
        
        if rover.x == destinationList[0] and rover.y == destinationList[1]:
            break


if __name__ == "__main__":
    main()
