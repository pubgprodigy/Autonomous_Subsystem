# Autonomous Subsystems, IIT-B Mars Rover Team
The repository for autonomous subsytems codes for Mars Rover Project, IIT-Bombay

* [GPS_IMU_app](./GPS_IMU_app): Contains code for android app made to publish GPS and IMU sensor readings of an android phone (preferably motorola) as rostopics `\LatLon` and `\IMU`
* [Mobility Workspace](./mobility_ws): Contains code for closed loop steer of rover for level 3 autonomous implementation
* [Steer-Dive Switch](./steer_drive_switch.py): **Python2.7** code for shifting between steer and drive modes (For Testing purposes)

-----------------------------------------------------------------------------------

- [ ] Ball detection using GMM
- [ ] Ball detection using CNN (in correspondence with Vito)
- [ ] Illumination correction
- [ ] Stereo_image_proc ros package testing
- [ ] GUI integration with python

Message definition of sensor_msgs/LaserScan.msg: 

Header header

float32 angle_min
float32 angle_max
float32 angle_increment

float32 time_increment

float32 scan_time # Time between scans

float32 range_min # maximum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data - (Note: values < range_min or > range_max should be discarded)    
float32[] intensities # intensity data [device-specific units]
