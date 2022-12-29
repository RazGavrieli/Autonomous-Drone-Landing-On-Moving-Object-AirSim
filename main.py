from algorithm_functions import *
import timeit 

if __name__ == "__main__":
    drone = autonomous_landing('LandingTarget_2', time.sleep, (5), time_to_leash=10)
    time.sleep(2)
    drone.RUN()
    
    logger.info("drone is disarmed! landed")