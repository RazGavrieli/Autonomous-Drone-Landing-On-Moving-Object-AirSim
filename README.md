# Autonomous-Drone-Landing-On-Moving-Object-AirSim
Easily land the drone on a moving or stationary target after the mission has completed! <br>

### HOW TO USE
from autonomous_landing import autonomous_landing
The object autonomous_landing(target_name, mission_function, mission_function_args) gets a name of the target, a function that you've created and want the drone to complete and the function's arguments if there are any. <br>
#### usage example:
```
drone = autonomous_landing('LandingTarget_2', time.sleep, (5))
     drone.RUN()
```
In this case, the drone will complete a sleep function, for 5 seconds. You can create your own function with more advanced controls. <br>
After completing the mission, the drone will search for a target named 'LandingTarget_2' and will complete an autonomous landing on that target. <br> (target name is from unreal)

<br> <br>
Tested on airsim for unreal 4.27. <br> 
Runs on blocks project (deafult project for airsim) <br>
