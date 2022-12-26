from algorithm_functions import *
import timeit 

if __name__ == "__main__":
    disarm()
    arm()
    takeoff()
    mission(2)
    direction = GET_DIRECTION_OF_TARGET()
    LEASH_TIME = 20 # seconds
    i = 0
    while i<LEASH_TIME: 
        dist = leashTracking() # takes one second
        if dist < 35:
            i+=1 
    above = False
    while not above:
        above = helipadApproach()

    while client.getMultirotorState().gps_location.altitude > 125:
        finalApproach()
    
    while not touchdown():
        continue
    logger.info("drone is disarmed! landed")