# hand2eyecalib
Different Solvers for AX=XB problem forumation of hand2eye calibratiob i.e A fixed camera and calibration tag attached to the end-effector. Also a RANSAC scheme to choose best A,B pairs. 


## Solvers Available 
- Daniilidis1999
- ParkBryan1994
- TsaiLenz1989



## Dependencies
- [aruco_detect](http://wiki.ros.org/aruco_detect)
- libfranka
- python zmq

for data collection in simulation 
- franka_ros 
- gazebo 

## Python Dependenices:
- scipy

## Usage 

### Collect data: 
With real real robot:

- build and run server in the computer connected to the robot- either NUC/microPC 
```
cd <path to where you cloned this repo>/libfranka_NUC/
mkdir build 
cd build 
cmake ..
make 
./franka_control
```

- run aruco detection node, we are using aruco tags with dictionary (DICT_4X4_1000) and fiducial_len as 0.2m
```
roslaunch aruco_detect aruco_detect_calib.launch 
```
- run the data collection script
```
python CollectData.py
```
move the end effector ~10 poses and press enter at each pose to store data for calibration. 



with simulation: 
- launch simulation 

```
roslaunch franka_gazebo panda.launch x:=-0.5 \
world:=/home/ruthz/utm/base_ws/src/franka_ros/franka_gazebo/world/stone.sdf \
controller:=cartesian_impedance_example_controller \
rviz:=true
```

- run aruco detection node
```
roslaunch aruco_detect aruco_detect_calib.launch 
```
- run data collection script 
```
python SimCollectData.py
```
move the end effector using the interactive marker to ~10 poses and press enter at each pose to store data for calibration.

### Calibrate: 
Enter the correct data file's path in the Calibrate.py script and run

```
python Calibrate.py
```
### Notes: 
- If you have an error with Yaml as- 
```
TypeError: load() missing 1 required positional argument: 'Loader'
```

edit this line in "/opt/ros/noetic/lib/python3/dist-packages/tf/listener.py"
```
data = yaml.load(self._buffer.all_frames_as_yaml()) or {}
```
to 
```
data = yaml.safe_load(self._buffer.all_frames_as_yaml()) or {}
```
## References: 
-  Kronecker Product- https://github.com/eayvali/Pose-Estimation-for-Sensor-Calibration

