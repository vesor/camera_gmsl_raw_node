# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_video_rectifier_sample Video Rectification sample

The Video Rectification sample demonstrates how to remove fisheye distortion
from a video captured on a camera with a fisheeye lens.

![Fisheye video and undistorted version](sample_rectifier.png)

The sample reads frames from a video input recorded from an 180 fov camera and takes the 
calibration from the `rig.xml` file. It then performs rectification and displays
both the original and rectified video side-by-side.

    ./sample_video_rectifier

To play a custom video and with a corresponding rig calibration file, the options `--video` and `--rig` can be used:

    ./sample_video_rectifier --video=<video file.h264> --rig=<rig.xml>

It is possible to change the field of view the output by using `--fovX=0..n` and `--fovY=0..n`. By default this is set
to 120 and 90 degrees respectively. This property is only part of pinhole cameras.

If the rig.xml contains more cameras, to select the camera index specify `--cameraIdx=0...n`, 0 by default

It is possible to take a screenshot of the rectified image by pressing S, or it is possible to specify
the flag `--dumpToFile=0/1` to save all rectified frames.
