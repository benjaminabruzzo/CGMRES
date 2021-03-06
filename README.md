# Introduction
This project provides Continuation/GMRES method (C/GMRES method), a fast algorithm of NMPC, and the multiple shooting based C/GMRES method.


# Requirement
- C++11
- Cmake
- Eigen 3
- Python3, numpy, seaborn (for plotsim.py and 2linkanimation.py)
- ffmpeg (to save animations generated by 2linkanimation.py)


# install

	mkdir -p ~/mayataka/cmgres && cd ~/mayataka/cmgres && git init
	git remote add gh git@github.com:benjaminabruzzo/CGMRES.git && git pull gh master && git fetch gh

# Usage
After setting the parameters of your model of NMPC in nmpc_model.hpp, writing equations in nmpc_model.cpp, and setting simulation conditions in main.cpp, build files as follows:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```


After performing a numerical simulation, you can see graphs by

```
$ python3 plotsim.py example
```

save graphs by

```
$ python3 plotsim.py example save example
```

see an animation of 2link robot by

```
$ python3 2linkanimation.py example
```

and save its animation by

```
$ python3 2linkanimation.py example save example
```

mkdir -p ~/mayataka/cmgres && cd ~/mayataka/cmgres && git init


# References
1. T. Ohtsuka A continuation/GMRES method for fast computation of nonlinear receding horizon control, Automatica, Vol. 40, No. 4, pp. 563-574 (2004)
2. C. T. Kelly, Iterative methods for linear and nonlinear equations, Frontiers in Apllied Mathematics, SIAM (1995)
3. Y. Shimizu, T. Ohtsuka, M. Diehl, A real‐time algorithm for nonlinear receding horizon control using multiple shooting and continuation/Krylov method, International Journal of Robust and Nonlinear Control, Vol. 19, No. 8, pp. 919-936 (2008)
