# AutonomousWheelchair
Navigation and localisation for an autonomous wheelchair

### Test Out the Stereo Camera
The `wheelchair_description` package contains a differential drive robot with a multicamera plugin that simulates a stereo camera in gazebo.
To launch the robot:
`roslaunch wheelchair_description gazebo.launch`

To visualise the camera feed, run (in another terminal):
`rosrun image_view stereo_view stereo:=/stereo image:=image_raw _approximate_sync:=True`

And (in another terminal):
`ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc`

To transform the stereo camera input into the appropriate frame (in another terminal):
`rosrun tf2_ros static_transform_publisher 0 0 1.05 1.57 0 -1.57  base_link sensor_link`

To run the mapping node run:
`roslaunch hector_mapping mapping_default.launch'

To save the map run:
`rosrun map_server map_saver`

## Future steps and prospectus

### Motor Driver Vs Motor Controller

#### Motor Controller

#### Motor driver

