# # You may need to import some classes of the controller module. Ex:
# #  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
# # import numpy as it may be used in future labs
# import numpy as np

# #######################################################
# # Creates Robot
# #######################################################
# robot = Robot()


# #######################################################
# # Sets the time step of the current world
# #######################################################
# timestep = int(robot.getBasicTimeStep())

# #######################################################
# # Gets Robots Distance Sensors
# # Documentation:
# #  https://cyberbotics.com/doc/reference/distancesensor
# #######################################################
# frontDistanceSensor = robot.getDevice('front distance sensor')
# leftDistanceSensor = robot.getDevice('left distance sensor')
# rightDistanceSensor = robot.getDevice('right distance sensor')
# rearDistanceSensor = robot.getDevice('rear distance sensor')
# frontDistanceSensor.enable(timestep)
# leftDistanceSensor.enable(timestep)
# rightDistanceSensor.enable(timestep)
# rearDistanceSensor.enable(timestep)

# #######################################################
# # Gets Robots Lidar Distance Sensors
# # Documentation:
# #  https://cyberbotics.com/doc/reference/lidar
# #######################################################
# lidar = robot.getDevice('lidar')
# lidar.enable(timestep)
# lidar_horizontal_res = lidar.getHorizontalResolution()
# lidar_num_layers = lidar.getNumberOfLayers()
# lidar_min_dist = lidar.getMinRange()
# lidar_max_dist = lidar.getMaxRange()


# print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
# print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

# #######################################################
# # Gets Robots Camera
# # Documentation:
# #  https://cyberbotics.com/doc/reference/camera
# #######################################################
# camera = robot.getDevice('camera1')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)

# #######################################################
# # Gets Robots Motors
# # Documentation:
# #  https://cyberbotics.com/doc/reference/motor
# #######################################################
# leftMotor = robot.getDevice('left wheel motor')
# rightMotor = robot.getDevice('right wheel motor')
# leftMotor.setPosition(float('inf'))
# rightMotor.setPosition(float('inf'))
# leftMotor.setVelocity(0)
# rightMotor.setVelocity(0)


# #######################################################
# # Gets Robot's the position sensors
# # Documentation:
# #  https://cyberbotics.com/doc/reference/positionsensor
# #######################################################
# leftposition_sensor = robot.getDevice('left wheel sensor')
# rightposition_sensor = robot.getDevice('right wheel sensor')
# leftposition_sensor.enable(timestep)
# rightposition_sensor.enable(timestep)

# #######################################################
# # Gets Robot's IMU sensors
# # Documentation:
# #  https://cyberbotics.com/doc/reference/inertialunit
# #######################################################
# imu = robot.getDevice('inertial unit')
# imu.enable(timestep)

# # Main loop:
# # perform simulation steps until Webots is stopping the controller

# while robot.step(timestep) != -1:
#     # Read the sensors:
#     # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
#     full_range_image = lidar.getRangeImage()
#     # print size of Range Image
#     print('#################################################################')
#     print("Lidar's Full Range Image Size: ", len(full_range_image))
#     # Compare Distance Sensors to Lidar Ranges
#     front_dist = frontDistanceSensor.getValue()
#     right_dist = rightDistanceSensor.getValue()
#     rear_dist = rearDistanceSensor.getValue()
#     left_dist = leftDistanceSensor.getValue()

#     print("Distance Sensor vs Lidar")
#     print("\tFront:\t", front_dist, "\t|", full_range_image[0])
#     print("\tRight:\t", right_dist, "\t|", full_range_image[90])
#     print("\tRear:\t", rear_dist, "\t|", full_range_image[180])
#     print("\tLeft:\t", left_dist, "\t|", full_range_image[270])

#     # camera object recognition
#     obj_in_view = len(camera.getRecognitionObjects())
#     print("Objects in View: ", obj_in_view)

#     if (obj_in_view > 0):
#         pos_image = camera.getRecognitionObjects()[0].getPositionOnImage()[0]
#         pos_view = camera.getRecognitionObjects()[0].getPosition()[0] 
        
#         print("\tPosition in Image:\t", pos_image) # in pixels relative to image
#         print("\tPosition in View:\t", pos_view) # in meters relative to camera
          
#     # Enter here functions to send actuator commands, like:
#     leftMotor.setVelocity(6)
#     rightMotor.setVelocity(6)

#     if full_range_image[0] < .07:

#         leftMotor.setVelocity(0)
#         rightMotor.setVelocity(0)
#         break
# # Enter here exit cleanup code.



"""
Matias Cinera - U 6931_8506
CAP-6626
Instructor: Dr.Alfredo Weitzenfeld 
Ta:         Chance Hamilton
Assigment:  lab3_task2 controller.
"""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math

#######################################################
# Creates Robot
#######################################################
robot = Robot()
#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())
#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()

print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)


# global variables
distBtwWhe = 2.28
dmid = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi
half_of_robot = 0.037*39.3701 

def getLidar():
    image = lidar.getRangeImage()
    toIn = 39.3701
    return [image[270]*toIn - half_of_robot, image[90]*toIn - half_of_robot]

# set speed to motors
def setSpeedIPS(vl, vr):
    vl /= w_r
    vr /= w_r
    leftMotor.setVelocity(vl)
    rightMotor.setVelocity(vr)

# saturation fnc
def v_saturation(v, max):
    if math.isinf(v):
        return max
    if v > max:
        return max
    if v < -max:
        return -max
    return v

# return the distance in inches from the front pid
def front_dist():
    return frontDistanceSensor.getValue()*39.3701

def front_lidar():
    image = lidar.getRangeImage()
    return (image[0]*39.3701) - half_of_robot

# assume angle is in radians
def rotationInPlace(direction, angle, v):
    s = angle*dmid
    time = s/v
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        if direction == "left":
            setSpeedIPS(-v, v)
        else:
            setSpeedIPS(v, -v)


def getLidarMin():
    image = lidar.getRangeImage()
    toIn = 39.3701

    min_left = 999
    min_right = 999

    for i in range(270, 360):
        if min_left > image[i]:
            min_left = image[i]
    
    for i in range(0, 91):
        if min_right > image[i]:
            min_right = image[i]

    return [min_left*toIn - half_of_robot, min_right*toIn - half_of_robot]


def wallFollowLidar(wall, flid, k):
    lids = getLidarMin()

    left_lid = lids[0]
    right_lid = lids[1]
    dist_to_wall = 2.5
    v = v_saturation(flid, 4)
    error = (v - 2.5)  # target distance to wall = 2.5 inches
    pid = v-abs(error)*k
    if wall == 'right':    
        if flid > 3:
            if right_lid < dist_to_wall:    # too close to target wall
                setSpeedIPS(pid, v)
            elif right_lid > dist_to_wall:  # too far to target wall
                setSpeedIPS(v, pid)
            elif left_lid < dist_to_wall:   # too close to opposite wall
                setSpeedIPS(v, pid)
        else:
            setSpeedIPS(v, v)
    elif wall == 'left':
        if flid > 3:
            if left_lid < dist_to_wall:    # too close to target wall
                setSpeedIPS(v, pid)
            elif left_lid > dist_to_wall:  # too far to target wall
                setSpeedIPS(pid, v)
            elif right_lid < dist_to_wall: # too close to opposite wall
                setSpeedIPS(pid, v)
        else:
            setSpeedIPS(v, v)

def wallFollow():
    kps_vals = [0.1, 0.5, 1.0, 2.0, 2.5, 5.0]
    fpid = front_dist()
    wall = 'right'
    if fpid < 2.5:  # to close to wall, rotate 45 deg away from it
        rotationInPlace('left', pi/4, 0.9)
    else:   # else follow wall
        wallFollowLidar(wall, front_lidar(), kps_vals[2])


def main():
    while robot.step(timestep) != -1:
        # wallFollow()
        pass

if __name__ == "__main__":
    main()