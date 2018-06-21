
# camera_gmsl_raw_ros

Ros node that reads images from camera and publishes it as a ros topic.

## Compile

The code should be compiled using catkin_make on target machine (Drive PX2).

Before compile this code, please crosscompile the project lib_camera_gmsl_raw on host machine. Then copy main.hpp and *.so into camera_node/external folder on target machine.

## Run

rosrun camera_node camera_node

# lib_camera_gmsl_raw

Provide API to read camera data on Drive PX2. Code is based on driveworks sample sample_camera_gmsl_raw, with just few lines of change.

## Compile

Compile on host linux machine using crosscompile.

# Common code based on nvidia driveworks samples

driveworks_samples folder is just copied from driveworks samples.

driveworks_samples/src/framework stays no change. 
