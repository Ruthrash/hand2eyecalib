# hand2eyecalib
Different Solvers for AX=XB problem forumation of hand2eye calibratiob i.e A fixed camera and calibration tag attached to the end-effector 


## Solvers Available 
- Daniilidis1999
- ParkBryan1994
- TsaiLenz1989
- Kronecker Product
- EKF 
- IEKF 


- Ceres:Levenberg-Marquadt 
- (Factor Graphs?)

## Dependencies
- franka_ros
- libfranka

## Python Dependenices:
- scipy
- baldor

## Install Ceres 
- https://brucknem.github.io/posts/install-ceres-solver/

## References: 
- EKF, IEKF, UKF, Kronecker Product- https://github.com/eayvali/Pose-Estimation-for-Sensor-Calibration
- closed form - https://github.com/crigroup/handeye
- ceres solver- https://github.com/jhu-lcsr/handeye_calib_camodocal
- 