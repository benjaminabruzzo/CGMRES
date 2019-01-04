# Introduction
This project provides Continuation/GMRES method (C/GMRES method), a fast algorithm of NMPC, and the multiple shooting based C/GMRES method.  It has been converted to run with ROS kinetic on Ubuntu 16.04.  No guarantees of compatibilities are made with any other version of anything :-)

## CAVEAT
This repository is still under development.


# Requirement
- C++11
- Cmake
- Eigen 3
- Python3, numpy, seaborn (for plotsim.py and 2linkanimation.py)
- ffmpeg (to save animations generated by 2linkanimation.py)

# Install
	mkdir -p ~/ros/src/mayataka_ros && cd ~/ros/src/mayataka_ros && git init
	git remote add gh git@github.com:benjaminabruzzo/CGMRES.git && git fetch gh
	git checkout ros-kinetic-ardrone
	
	cd ~/ros
	catkin build mayataka_ros

# Usage
	roslaunch mayataka_ros nmpc.launch

# Choosing between C/GMRES and Multiple Shooting C/GMRES
To chose which solver is used, there is only one places that code needs to be changed, just swap which line is commented:

src/mayataka_nmpc.cpp:

	// ContinuationGMRES cgmres_solver;
	MultipleShootingCGMRES cgmres_solver;


# References
1. T. Ohtsuka A continuation/GMRES method for fast computation of nonlinear receding horizon control, Automatica, Vol. 40, No. 4, pp. 563-574 (2004)
2. C. T. Kelly, Iterative methods for linear and nonlinear equations, Frontiers in Apllied Mathematics, SIAM (1995)
3. Y. Shimizu, T. Ohtsuka, M. Diehl, A real‐time algorithm for nonlinear receding horizon control using multiple shooting and continuation/Krylov method, International Journal of Robust and Nonlinear Control, Vol. 19, No. 8, pp. 919-936 (2008)
