import airsim

import logging as logger
import time
import math

# SET CONSTANTS
logger.basicConfig(level=logger.INFO)
MISSION_HEIGHT = 30
LEASH_DISTANCE = 30
TARGET_NAME = 'LandingTarget_2'
LEASH_SPEED = 15
APPROACH_SPEED = 4
LOWERING_SPEED = 1
DIRECTION_OF_TARGET = 0
SPEED_OF_TARGET = 0
b = 0

landingHeight = MISSION_HEIGHT-15
client = airsim.MultirotorClient() 

# TODO Calculate disttance to target
# TODO Calculate landing target height and in leashTracking() set X,Y to be according to distance
# TODO Find a solution to "wobbely" behaviour other than slowing the drone down

def disarm():
    """drone is before takeoff and is turned-off (may not be used)"""
    pass

def arm():
    """drone is before takeoff and is ready to takeoff"""
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    logger.info("Drone is armed")

def takeoff():
    """drone is taking off to a set height"""
    client.takeoffAsync().join()
    logger.info("Take off precedure complete, acending to mission height")
    client.moveToPositionAsync(0, 0, -MISSION_HEIGHT, velocity=LEASH_SPEED, drivetrain=0).join()
    logger.info("Drone is now at the constant mission height %f", MISSION_HEIGHT)

def mission(delay):
    """drone is doing another mission"""
    logger.info("Drone is in mission")
    time.sleep(delay)
    logger.info("Drone finished mission")

def GET_DIRECTION_OF_TARGET():
    global SPEED_OF_TARGET
    global DIRECTION_OF_TARGET
    global b
    x1, y1 = searchTarget()
    time.sleep(1)
    x2, y2 = searchTarget()

    DIRECTION_OF_TARGET = (y1-y2)/(x1-x2)
    b = y1 - DIRECTION_OF_TARGET*x1
    SPEED_OF_TARGET = math.dist([x1, y1], [x2, y2])
    if SPEED_OF_TARGET > 10:
        logger.error("TARGET TOO QUICK")
        exit(0)
    LEASH_SPEED = SPEED_OF_TARGET
    logger.info("target speed: %f", SPEED_OF_TARGET)

def searchTarget():
    """drone is searching for target (uses client.simGetObjectPose(targetName))"""
    pose = client.simGetObjectPose(TARGET_NAME)
    targetx = ''
    while type(targetx) != float:
        targetx, targety = pose.__dict__['position'].x_val, pose.__dict__['position'].y_val
    return targetx, targety

def leashTracking():
    """drone is keeping a constant distance from the target (dist)"""
    targetx, targety = searchTarget()

    dronex, droney = client.getMultirotorState().__dict__['kinematics_estimated'].position.x_val, client.getMultirotorState().__dict__['kinematics_estimated'].position.y_val # get x,y of drone
    currDistance = math.dist([targetx, targety], [dronex, droney])
    logger.info("drone x,y: %f ,%f | target x,y: %f ,%f", dronex, droney, targetx, targety)
    if currDistance < LEASH_DISTANCE:
        LEASH_SPEED = SPEED_OF_TARGET
    else:
        LEASH_SPEED = SPEED_OF_TARGET+3
    futurey = DIRECTION_OF_TARGET*(targetx+LEASH_DISTANCE) + b
    logger.info("drone got command to stay %d %s from target, go to: %f,\t %f, at speed %f, curr dist: %f", LEASH_DISTANCE, "DISTANCE UNITS", targetx+LEASH_DISTANCE, futurey, LEASH_SPEED, currDistance)

    client.moveToPositionAsync(targetx, futurey, -MISSION_HEIGHT, velocity=LEASH_SPEED, drivetrain=0)
    GET_DIRECTION_OF_TARGET() # sleep one second
    return currDistance

def centering():
    """drone is keeping it's cameras centered on target (may not be used)"""
    pass

def helipadApproach():
    """drone is closing the distance to the target (getting close until right above)"""
    logger.info("drone got command to be right above the target")
    targetx, targety = searchTarget()
    client.moveToPositionAsync(targetx, targety, -(MISSION_HEIGHT-15), velocity=SPEED_OF_TARGET+3, drivetrain=0)
    GET_DIRECTION_OF_TARGET() # sleep one second
    dronex, droney = client.getMultirotorState().__dict__['kinematics_estimated'].position.x_val, client.getMultirotorState().__dict__['kinematics_estimated'].position.y_val # get x,y of drone
    currDistance = math.dist([targetx, targety], [dronex, droney])
    return currDistance < 0.1


def guidingTargetCentering():
    """drone is keeping the second target centered (may not be used)"""
    pass

def gimbleAdjustment():
    """changing camera's position to center on second target (may not be used)"""
    pass

def finalApproach():
    """drone lowers attitude"""
    global landingHeight
    targetx, targety = searchTarget()
    logger.info("drone is lowering attitude right above target %f, %f", targetx,targety)
    
    client.moveToPositionAsync(targetx, targety, -landingHeight, velocity=SPEED_OF_TARGET+1, drivetrain=0)
    GET_DIRECTION_OF_TARGET() # sleep one second
    landingHeight -= LOWERING_SPEED

def touchdown():
    """disarming the drone when its right on target"""
    logger.info("disarming..")
    return client.armDisarm(False)

def fail_safe():
    logger.warning("mayday :(")
    client.hoverAsync().join()