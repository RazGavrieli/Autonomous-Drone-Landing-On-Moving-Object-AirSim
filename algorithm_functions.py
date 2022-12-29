import airsim

import logging as logger
import time
import math
logger.basicConfig(level=logger.INFO)


class autonomous_landing:

    def __init__(self, target_name: str, mission_method: callable, *mission_args , time_to_leash: float = 20, target_height: float = 125) -> None:
        # SET CONSTANTS
        self.LEASH_DISTANCE = 30
        self.LEASH_HEIGHT = 30
        self.TAKE_OFF_HEIGHT = 30
        self.COMMAND_TIME = 1

        # SET VARIABLES
        self.time_to_leash = time_to_leash/self.COMMAND_TIME
        self.target_name = target_name
        self.target_height = target_height
        self.mission_method = mission_method
        self.mission_args = mission_args
        self.landingHeight = self.LEASH_HEIGHT-15
        self.client = airsim.MultirotorClient() 
        
        # VARIABLES TO BE CALCULATED
        self.direction_of_target = 0
        self.speed_of_target = 0
        self.b = 0
        self.curr_dist_from_target = 0

    def RUN(self):
        self.disarm()
        self.arm()
        self.takeoff()
        self.mission()
        i = 0

        while i<self.time_to_leash: 
            self.leashTracking()
            if self.curr_dist_from_target < self.LEASH_DISTANCE+5:
                i+=1 

        while self.curr_dist_from_target >= 0.2:
            self.helipadApproach()

        while self.client.getMultirotorState().gps_location.altitude > self.target_height:
            print(self.curr_dist_from_target)
            self.finalApproach()

        while not self.touchdown():
            continue

    def disarm(self):
        """drone is before takeoff and is turned-off (may not be used)"""
        pass

    def arm(self):
        """drone is before takeoff and is ready to takeoff"""
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        logger.info("Drone is armed")

    def takeoff(self):
        """drone is taking off to a set height"""
        self.client.takeoffAsync().join()
        logger.info("Take off precedure complete, acending to mission height")
        dronex, droney = self.client.getMultirotorState().__dict__['kinematics_estimated'].position.x_val, self.client.getMultirotorState().__dict__['kinematics_estimated'].position.y_val # get x,y of drone
        self.client.moveToPositionAsync(dronex, droney, -self.TAKE_OFF_HEIGHT, velocity=15, drivetrain=0).join()
        logger.info("Drone is now at the constant mission height %f", self.TAKE_OFF_HEIGHT)

    def mission(self):
        """drone is doing another mission"""
        logger.info("Drone is in mission")
        self.mission_method(*self.mission_args)
        logger.info("Drone finished mission")

    def GET_DIRECTION_OF_TARGET(self):
        x1, y1 = self.searchTarget()
        time.sleep(self.COMMAND_TIME)
        x2, y2 = self.searchTarget()
        self.speed_of_target = math.dist([x1, y1], [x2, y2])/self.COMMAND_TIME


        self.direction_of_target = (y1-y2)/(x1-x2)
        self.b = y1 - self.direction_of_target*x1
        if self.speed_of_target > 10:
            logger.error("TARGET TOO QUICK")
            exit(0)
        logger.info("target speed: %f", self.speed_of_target)

    def searchTarget(self):
        """drone is searching for target (uses client.simGetObjectPose(targetName))"""
        pose = self.client.simGetObjectPose(self.target_name)
        targetx = ''
        while type(targetx) != float:
            targetx, targety = pose.__dict__['position'].x_val, pose.__dict__['position'].y_val
        return targetx, targety

    def leashTracking(self):
        """drone is keeping a constant distance from the target (dist)"""
        targetx, targety = self.searchTarget()
        dronex, droney = self.client.getMultirotorState().__dict__['kinematics_estimated'].position.x_val, self.client.getMultirotorState().__dict__['kinematics_estimated'].position.y_val # get x,y of drone
        self.curr_dist_from_target = math.dist([targetx, targety], [dronex, droney])
        logger.info("drone x,y: %f ,%f | target x,y: %f ,%f", dronex, droney, targetx, targety)
        if self.curr_dist_from_target < self.LEASH_DISTANCE:
            leash_speed = self.speed_of_target
        else:
            leash_speed = self.speed_of_target+3
        futurey = self.direction_of_target*(targetx+self.LEASH_DISTANCE) + self.b
        logger.info("drone got command to stay %d %s from target, go to: %f,\t %f, at speed %f, curr dist: %f", self.LEASH_DISTANCE, "DISTANCE UNITS", targetx+self.LEASH_DISTANCE, futurey, leash_speed, self.curr_dist_from_target)

        self.client.moveToPositionAsync(targetx, futurey, -self.LEASH_HEIGHT, velocity=leash_speed, drivetrain=0)
        self.GET_DIRECTION_OF_TARGET() # sleep self.COMMAND_TIME seconds

    def centering(self):
        """drone is keeping it's cameras centered on target (may not be used)"""
        pass

    def helipadApproach(self):
        """drone is closing the distance to the target (getting close until right above)"""
        logger.info("drone got command to go to be right above the target")
        targetx, targety = self.searchTarget()
        self.client.moveToPositionAsync(targetx, targety, -(self.LEASH_HEIGHT-15), velocity=self.speed_of_target+2, drivetrain=0)
        self.GET_DIRECTION_OF_TARGET() # sleep self.COMMAND_TIME seconds
        dronex, droney = self.client.getMultirotorState().__dict__['kinematics_estimated'].position.x_val, self.client.getMultirotorState().__dict__['kinematics_estimated'].position.y_val # get x,y of drone
        self.curr_dist_from_target = math.dist([targetx, targety], [dronex, droney])


    def guidingTargetCentering(self):
        """drone is keeping the second target centered (may not be used)"""
        pass

    def gimbleAdjustment(self):
        """changing camera's position to center on second target (may not be used)"""
        pass

    def finalApproach(self):
        """drone lowers attitude"""
        targetx, targety = self.searchTarget()
        logger.info("drone is lowering attitude right above target %f, %f", targetx,targety)
        
        self.client.moveToPositionAsync(targetx, targety, -self.landingHeight, velocity=self.speed_of_target+1, drivetrain=0)
        self.GET_DIRECTION_OF_TARGET() # sleep self.COMMAND_TIME seconds
        self.landingHeight -= 1
        

    def touchdown(self):
        """disarming the drone when its right on target"""
        logger.info("disarming..")
        return self.client.armDisarm(False)

    def fail_safe(self):
        logger.warning("mayday :(")
        self.client.hoverAsync().join()