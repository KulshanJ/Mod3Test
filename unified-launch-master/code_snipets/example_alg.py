from qset_lib import Rover
from time import sleep
import rospy
from qset_msgs.msg import *
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Twist
from qset_msgs.msg import wheelSpeed
from sensor_msgs.msg import LaserScan
import math
from tf.transformations import euler_from_quaternion

def __modelstates_callback(msg):
        
    for name, pose in zip(msg.name, msg.pose):
            if name == self.__name:
                self.x = pose.position.x
                self.y = pose.position.y

                heading = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                      pose.orientation.w])[2] / math.pi * 180.0
                
                return heading


def main(msg):
    
    rover = Rover()
    
    i = 0
       
    #heading = rover.heading

    left_side_speed = 7
    right_side_speed = -7

    while i < 1000:
        #print("X: " + rover.x + " Y: " + rover.y + " Heading: " + rover.heading)

        for dist in rover.laser_distances:
            if dist < 2:
                    #left_side_speed = -7
                    #right_side_speed = -7
                    print(__modelstates_callback(msg))
                    

                
        rover.send_command(left_side_speed, right_side_speed)
        i = i + 1
        sleep(0.01)


if __name__ == "__main__":
    main(msg)
