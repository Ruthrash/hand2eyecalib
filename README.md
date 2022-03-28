# hand2eyecalib
Different Solvers for AX=XB problem forumation of hand2eye calibratiob i.e A fixed camera and calibration tag attached to the end-effector 


## Solvers Available 
- Daniilidis1999
- ParkBryan1994
- TsaiLenz1989

- EKF 
- IEKF 
- Ceres:Levenberg-Marquadt 
- (Factor Graphs?)

## Dependencies
- Ceres C++ 
- franka_ros
- libfranka

## Python Dependenices:
- PyRansac - https://pyransac.readthedocs.io/en/latest/
- scipy
- baldor

## Install Ceres 
- https://brucknem.github.io/posts/install-ceres-solver/

## References: 
- EKF, IEKF, UKF, Kronecker Product- https://github.com/eayvali/Pose-Estimation-for-Sensor-Calibration
- find references for solvers
- ceres solver- https://github.com/jhu-lcsr/handeye_calib_camodocal
- 