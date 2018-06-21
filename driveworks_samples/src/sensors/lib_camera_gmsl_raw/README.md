# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_gmsl_raw_sample RAW GMSL Camera Capture Sample

The RAW GMSL Camera Capture sample uses the GMSL camera interface available on
NVIDIA DRIVE<sup>&trade;</sup> PX 2 platforms. The sample creates a file for the RAW
data from the first camera on the selected CSI-port. The file is written to the disk.

![RAW GMSL Camera Capture Sample](sample_camera_gmsl_raw.png)

## Setting Up Cameras

For information about the physical location of the ports on the NVIDIA DRIVE
platform, see "Camera Setup under Configuration and Setup" in _NVIDIA DRIVE 5.0 Linux
PDK Development Guide_.

## Running the Sample

A camera must be connected to one of the NVIDIA DRIVE PX 2 CSI-ports. 

The following options are supported:
- `--csi-port={ab,cd,ef}` specifies a CSI port. Default value: `ab`.
- `--camera-type` specifies the type of the camera. 
   Supported cameras are`ar0231-rccb` (`ar0231-rccb-ss3322`, `ar0231-rccb-ss3323`), `ar0231-rccb-bae` and `ar0231-rccb-ssc`.
- `--write-file` specifies the output file. If `write-file` is not provided, no file is written out on disk.

If the camera type is `ar0231-rccb-ssc` and the `csi-port` is the default ab, enter:

    ./sample_camera_gmsl_raw --write-file <filename>

Otherwise, enter a command such as:

    ./sample_camera_gmsl_raw --csi-port=cd --camera-type=ar0231-rccb --write-file <filename>

If camera capture is running, i.e., no error occurred, the sample prints on the
console:

    First 16 bytes from top data lines:
    80 2 82 2a 0 8 42 29 0 0 81 16 0 0 81 16

    First 16 bytes from bottom data lines:
    10 0 10 0 10 0 10 0 10 0 10 0 10 0 10 0

    Exposure Time (s): 0.000658364

You can specify a demosaicing option with the `--interpolationDemosaic` option.
The following values are supported:
- 0 : half camera resolution is used.
- > 0 : full camera resolution with interpolation is used.
The argument `serializer-type` must be `uncompressed`.

Press S to take a 16 bit png snapshot of the current RAW frame.
 
## Run on Tegra B

#### Master Mode Prerequisites

Before running camera applications only on Tegra B, you must disable FRSYNC and
the forward/reverse control channel of the Tegra A aggregator. For related information,
see *Camera Setup (P2379)* in *NVIDIA DRIVE Linux 5.0 PDK Development Guide*.
This guide explains how to:
* Turn on the MAX9286 aggregator.
* Disable FRSYNC and the forward/reverse control channel on MAX9286 aggregator
  to avoid any interference with MAX96799.

#### Master Mode

After you have addressed the prerequisites (above) and have rebooted Tegra B,
you can run camera applications on Tegra B.

@note Running camera applications on Tegra A re-enables the forward/reverse control channel and FRSYNC from MAX9286
and the procecedure of activating camera for Tegra B need to be repeated.

#### Slave mode
Cameras can be captured on Tegra B in slave mode, i.e. when they are already captured
by an application on Tegra A. In such case it is possible to specify the "slave" flag, which can
be 0 or 1. If slave is 1, then Tegra B will not be able to run cameras
autonomously but it requires that camera to run at the same time from Tegra A.
If slave is false, then Tegra B can control any camera that is not currently
being used by Tegra A.
