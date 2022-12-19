from algorithm_functions import *

if __name__ == "__main__":
    disarm()
    arm()
    takeoff()
    mission(5)
    i = 0
    while i < 20:
        i+=1
        leashTracking()
    while i < 30:
        i+=1
        helipadApproach()
    while client.getMultirotorState().gps_location.altitude > 125:
        i+=1
        finalApproach()
    
    while not touchdown():
        pass
    logger.info("drone is disarmed! landed")