# AutonomousWheelchair
Navigation and localisation for an autonomous wheelchair

###Test Out the Stereo Camera
The `wheelchair_description` package contains a differential drive robot with a multicamera plugin that simulates a stereo camera in gazebo.
To launch the robot:
`roslaunch wheelchair_description gazebo.launch`

To visualise the camera feed, run (in another terminal):
`rosrun image_view stereo_view stereo:=/stereo image:=image_raw _approximate_sync:=True`

And (in another terminal):
`ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc`