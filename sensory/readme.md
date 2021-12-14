# Sensory on the simulator
This is a brief guide on how to let our work run with the simulator.
## Let the sensors run and sensor fusion
Run the following commands in no particular order:
- To run the stereo camera: ```rosrun sensory rgb_camera.py```
- To run the lidar: ```rosrun sensory lidar.py```

For both the stereo camera and the lidar you can use ```--visualize``` to show the how the cone detection is working, **beware that this could be very resource expensive**.

**Only after you've run both previous commands** you can run the following one for the sensor fusion, otherwise it won't work correctly:

To run the sensor fusion, type: ```rosrun sensory output_fusion.py```

If you want to visualize the real-time cone recognition, as for the lidar and stereocamera, use ```--visualize``` at the end of the aforementioned command. The sensor fusion won't show the same things lidar and the stereocamera, but it will print every cone that has been detected and labeled.

## Human interaction
As you now from the rules of the race, the car can start running only after it receives a "start" command. The following commands publish a string in the topic named ```/human_interaction``` whose value can be either _"start"_ or _"stop"_, the former lets the car start, while the latter suddenly stops the car in case of emergency.
- To let the car run: ```rosrun sensory human_interaction.py start```
- In case you want to stop the car: ```rosrun sensory human_interaction.py stop```

## Change the confidence of the models
- To change the confidence of the lidar model: ```rostopic pub /lidar/confidence std_msgs/Float32 <float_value>```
- To change the confidence of the camera model: ```rostopic pub /camera/confidence std_msgs/Float32 <float_value>```

**Important:** The _<float_value>_ should be within the range [0, 1].
