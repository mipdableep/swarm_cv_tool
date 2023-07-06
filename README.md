# swarm cv tool

collects video stream from tello and sends real time absulute location and peers detected

## Publishes to Topics

1. "location"
1. "peer_detection"

## Usage

In the following example, we use a parameter file located in the working directory.

```console
ros2 run swarm_env_detector location_peer_detection --params-file default_params.yaml
```

## Parameters

| Parameter 	| Meaning		|Default Value	|	Remark	|
| ---------		| ------------	|-------		|-------	|
| swarm_cv_tool.video_ip	| ip of the tello streaming video	| 0.0.0.0 | - |
| swarm_cv_tool.video_port	| video port for the tello stream	| 11111	| -	|
| swarm_cv_tool.calib_path	| yaml calibration file path    | /ros_ws/src/swarm_cv_tool/drone_calib/drone20.yaml	| -	|
| use_sim_time	| Should the time also be simulated	| false	|	See [ROS Clock](http://wiki.ros.org/Clock) |

## Dependencies

* [interfaces_alate](https://github.com/halehaka/interfaces_alate)
* [interfaces_swarm](https://github.com/halehaka/interfaces_swarm)
* OpenCV-contrib: Aruco module
