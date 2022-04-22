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
        left_side_speed = 5
        right_side_speed = -5
        rover.send_command(left_side_speed, right_side_speed)
        sleep(0.1)
    # if ccw
    else:
        # calculate the new desired vector
        heading = rover.heading
        heading = angleConvertion(heading)
        
        desiredHeading = heading + angle
        # turn right wheel forwards and left backwards
        left_side_speed = -5
        right_side_speed = 5
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

    listOfDifference = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    i = 0
    while i < 30:
      listOfDifference[i]=listFromLiDAR[i] - listOfAlertDistance1[i]
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
'''
def distanceChecking2(listOfAlertDistance2, listFromLiDAR, flag):
    
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






def stop_check(closestDistance):
    
    rover = Rover()
    # variables for sleep time, speed, stopping distance, minimum tolerated distance
    speed = 1
    sleeptime = 2
    closestdist = 8
    mindist = 2

    # call on lidar function to determine distance from object
    objectdist = closestDistance #not to be confused with closestdist... this calls on Sean's function
    if objectdist <= closestdist:
            left_side_speed = -7
            right_side_speed = -7
            rover.send_command(left_side_speed, right_side_speed)

            # call on turning function to rotate 45 degrees ccw
            turningFunction(-45)
            # call on lidar function to determine distance from object
            if closestDistance < 10:
                ccw45dist = closestDistance
            else:
                ccw45dist = 10






#BACKUP

#reversing
def backup1() :
    
    rover = Rover()

    i = -5
    j = -5
    left_side_speed = i
    right_side_speed = j

    rover.send_command(left_side_speed, right_side_speed)

    #call Eden's function
    if __name__ == "__stop_check__":
        stop_check()

initialHeadingVector = 0

#turning
def backup2() :

    rover = Rover()
    
    while 1:

        i = 5
        j = -5

        left_side_speed = i
        right_side_speed = j

        rover.send_command(left_side_speed, right_side_speed)

        #rotate 45 ccw, check if initial/final heading vectors equal

        if initialHeadingVector == finalHeadingVector :
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

    while 1 :

        sleeptime = 5

        left_side_speed = 5
        right_side_speed = 5

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
    
    destinationx = 15
    destinationy = 15
    
    theta = math.atan2(destinationy - rover.y, rover.x - destinationx)
    
    theta = theta * 57.2957795130823209
    
    print(theta)
    
    x = theta - 5
    y = theta + 5
    
    check = 1
    
    left_side_speed = 5
    right_side_speed = -5
    rover.send_command(left_side_speed, right_side_speed)
    
    while check == 1:
        realheading = rover.heading
        left_side_speed = 5
        right_side_speed = -5
        rover.send_command(left_side_speed, right_side_speed)
        if realheading < y and x < realheading:
            left_side_speed = 0
            right_side_speed = 0
            rover.send_command(left_side_speed, right_side_speed)
            check = 0
 
    #Step2: Move forward and scanning before bumping into obstacles
   
    left_side_speed = 5
    right_side_speed = 5
    rover.send_command(left_side_speed, right_side_speed)
    
    sleep(3.01)

    while i < 3000:
        rover.send_command(left_side_speed, right_side_speed)

        #flag = 0
        
        listOfLiDAR = [100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100] 
        k = 0

        for dist in rover.laser_distances:
            if dist<100:
                listOfLiDAR[k] = dist
                k=k+1
            
           
        closestDistance, result = distanceChecking1(listOfDistance1, listOfLiDAR ) 
        #flag = distanceChecking2(listOfDistance2, listOfLiDAR, flag)
        if result == True:
            stop_check(closestDistance)
        #elif flag > 1:
            #Call Lucas's function
        
        
            i = i + 1
        check = 1
        while check == 1:
            realheading = rover.heading
            left_side_speed = 3
            right_side_speed = -3
            rover.send_command(left_side_speed, right_side_speed)
            if realheading < y and x < realheading:
                left_side_speed = 0
                right_side_speed = 0
                rover.send_command(left_side_speed, right_side_speed)
                check = 0
        sleep(0.01)
        
        if rover.x == destinationList[0] and rover.y == destinationList[1]:
            break


if __name__ == "__main__":
    main()
