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

The wheelchair currently deployed a Roboteq XDC 2430 to communicate with ROS over ROSSerial. Motor controller however are firmware dependant and are subject to firmware updates which can over time cause in issue with compatibility. Over time any motor controller will become obsolete because of the firmware updates not being able to properly keep up with the old style of hardware that the controller would have.

#### Motor driver

Going forward , it's best to use a motor driver instead. A basic H-bridge motor driver will not host it's own firmware but will simple interact with motors on the wheelchair. In order to make it work, one needs to write their own ROSSerial package to enable sending twist messages to the motors via ROS.

The motors are equipped with their own set of encoders and are high torque geared DC motors which makes finding a suitable controller / driver for them slightly tricky as well.

Two options that can be opted for which will be ideal are:
Solo Mini breakout board: https://www.robotshop.com/ca/en/solo-mini-40v-80a-dc-bldc-pmsm-acim-motor-controller.html
O-Drive controller: this board is meant and used for BLDC motors however, if can be made to work with the motors in wheelchair would be really ideal. Helps bypass encoders from the motors.

Some choices for motor controllers if opted:
Roboteq MDC 2460: https://www.robotshop.com/ca/en/solo-mini-40v-80a-dc-bldc-pmsm-acim-motor-controller.html
MCP 263: https://www.robotshop.com/ca/en/mcp263-dual-60a-34vdc-advanced-motor-controller.html

A summary of future steps

Sensor POV:
Setup CUDA 11 and set up zed camera with ROS. --> port the slope prediction done with intel depth cameras to a ROS node which uses the zed camera. --> Discuss and try the possibilities to use a laser scanner or a sensor which might help more in detecting obstacles not present in the map ( local planner avoidance)

ROS POV:
Setup and implement a ROS Serial package --> make sure that twist messages can be sent to the wheelchair via this package --> make a basic hardware interface based on differential drive kinematics of the wheelchair to ensure accurate translation of messages into hardware --> Explore the different local planner possibilities for local obstacle avoidance --> Attempt to transform depthimage to laserscan within gazebo simulations and try navigation with the transformed topic.

Hardware POV and parallel things to look into:
Decide on the motor driver/ controller --> find a way to separately interface the encoders of the motors using microROS -- take out the old XDC 2430 controller on the wheelchair and replace the original propreitary controller of the Quickie Xperience 2. -- Make MT connectors for charging the lead acid batteries on the wheelchair.


Feel free to connect with any of the three in case of any doubts, we'd be happy to help in any way possible:

Raghav Thakar 
Akshat Pandey
Pierre Lamart

