import airsim

import logging as logger
import time

# SET CONSTANTS
logger.basicConfig(level=logger.INFO)
MISSION_HEIGHT = 30
LEASH_DISTANCE = 10
TARGET_NAME = 'LandingTarget_2'
LEASH_SPEED = 15
FIRST_APPROACH_SPEED = 4
FINAL_APPROACH_SPPED = 3
LOWERING_SPEED = 2

landingHeight = 30
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

def searchTarget():
    """drone is searching for target (uses client.simGetObjectPose(targetName))"""
    pose = client.simGetObjectPose(TARGET_NAME)
    x, y = pose.__dict__['position'].x_val, pose.__dict__['position'].y_val
    return x, y

def leashTracking():
    """drone is keeping a constant distance from the target (dist)"""
    logger.info("drone got command to say %d %s from target", LEASH_DISTANCE, "DISTANCE UNITS")
    x, y = searchTarget()
    client.moveToPositionAsync(x+LEASH_DISTANCE, y, -MISSION_HEIGHT, velocity=LEASH_SPEED, drivetrain=0).join()

def centering():
    """drone is keeping it's cameras centered on target (may not be used)"""
    pass

def helipadApproach():
    """drone is closing the distance to the target (getting close until right above)"""
    logger.info("drone got command to be right above the target")
    x, y = searchTarget()
    client.moveToPositionAsync(x, y, -MISSION_HEIGHT, velocity=FIRST_APPROACH_SPEED, drivetrain=0).join()

def guidingTargetCentering():
    """drone is keeping the second target centered (may not be used)"""
    pass

def gimbleAdjustment():
    """changing camera's position to center on second target (may not be used)"""
    pass

def finalApproach():
    """drone lowers attitude"""
    global landingHeight
    logger.info("drone is lowering attitude right above target")
    x, y = searchTarget()
    client.moveToPositionAsync(x, y, -landingHeight, velocity=FINAL_APPROACH_SPPED, drivetrain=0).join()
    landingHeight-=LOWERING_SPEED

def touchdown():
    """disarming the drone when its right on target"""
    logger.info("disarming..")
    return client.armDisarm(False)

def fail_safe():
    logger.info("mayday :(")
    client.hoverAsync().join()