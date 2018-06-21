# camera node

camera node reads images from camera and publishes it as a ros topic.

The code should be compiled using catkin_make on target machine (Drive PX2), no need crosscompile.

Before compile this code, please crosscompile the project lib_camera_gmsl_raw on host machine. Then copy main.hpp and *.so into camera_node/external folder on target machine.


