In order, run the following commands:

## Let sensors run
To run the stereo camera, type: `rosrun sensory rgb_camera.py` 
To run the lidar, type: `rosrun sensory lidar.py` 
To run the sensor fusion, type: `rosrun sensory routput_fusion.py`

## Human interaction
As you now from the rules of the race, the car can start to run only after it receives a "start" command. <br \>
To do that, type: `rosrun sensory human_interaction.py start`.
In case you want to suddenly stop the car, type: `rosrun sensory human_interaction.py stop`
