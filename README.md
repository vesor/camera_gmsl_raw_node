
# camera_gmsl_raw_ros

Ros node that reads images from camera and publishes it as a ros topic.

## Compile

The code should be compiled using catkin_make on target machine (Drive PX2).

Before compile this code, please crosscompile the project lib_camera_gmsl_raw on host machine. Then copy main.hpp and *.so into camera_node/external folder on target machine.

NOTE: If you want to modify the code, make sure main.hpp contains no API (such as std::string) that will break ABI compatible between GCC4.9 and GCC5.x, because the liblib_camera_gmsl_raw.so file is compiled using nvidia's GCC4.9 and catkin_make use GCC5.x

## Run

rosrun camera_node camera_node

Then you can use rosrun image_view image_view image:=/camera/image to view the rostopic

# lib_camera_gmsl_raw

Provide API to read camera data on Drive PX2. Code is based on driveworks sample sample_camera_gmsl_raw, with just few lines of change.

## Compile

Compile on host linux machine using crosscompile.

# Common code based on nvidia driveworks samples

driveworks_samples folder is just copied from driveworks samples.

driveworks_samples/src/framework stays no change. 
