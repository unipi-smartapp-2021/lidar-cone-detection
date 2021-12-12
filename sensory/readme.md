## Let sensors run
In order, run the following commands:

1)To run the stereo camera, type: `rosrun sensory rgb_camera.py` <br/>
2)To run the lidar, type: `rosrun sensory lidar.py` <br/>
**3**)To run the sensor fusion, type: `rosrun sensory routput_fusion.py` <br/>

## Human interaction
As you now from the rules of the race, the car can start to run only after it receives a "start" command. <br/>
To do that, type: `rosrun sensory human_interaction.py start` <br/>
In case you want to suddenly stop the car, type: `rosrun sensory human_interaction.py stop` <br/>

## Change the confidence of the models

To change the confidence of the lidar model, type: `rostopic pub /lidar/confidence std_msgs/Float32 <float_value>` <br/>
To change the confidence of the camera model, type: `rostopic pub /camera/confidence std_msgs/Float32 <float_value>` <br/>

**Important:** The <float_value> should be within the range [0,1].
