# pendulumRbdlMuscod
A simple example of a minimization OCP and a least squares OCP where the model is made using RBDL and the problem is numerically solved using MUSCOD-II.

# Tested Configurations

* Ubuntu 18.04.4 LTS
* RBDL 3.0.0, git commit: 4e36ba8a0fb9b68b45c25ad0fe40fc261bb3f351.
* MUSCOD-II, mercurial changeset:   186:326e1643e8c4

# Quick Start

1. Ensure that you have the following prerequisites, cloned, built, and installed:
  * RBDL built with the luamodel addon.  (https://github.com/ORB-HD/rbdl-orb). 
  * MUSCOD-II

2. Make a build folder within pendulumRbdlMuscod

3. Run cmake (or ccmake) from the 'build' folder. You will be asked to set CUSTOM\_RBDL\_PATH, which is the path to the install folder of the version of RBDL that you would like to use. If you have a system-wide installation of RBDL this folder is '/usr/local'

4. From a terminal in the build directory call 
  ```
    make
  ```    

5. Run the minimization problem
```
muscod_release pendulumMIN
```
  * The screen will output information related to the model, a numerical check, and then it will wait for you to press a key. The information ends with these lines:


> Dat file name            : DAT/pendulumMIN.dat
> Problem Type             : Minimization
> Model file               : ../DAT/pendulum.lua
> LSQ target states file   : ../DAT/lsqDataToTrack.csv
> LSQ Reg. Weighting: 0.001
>
> ----------------------------------------
> Numerical check of the model: 
>   Setting the model state and evaluating its generalized accelerations
> -1.570796 q
> 0.000000  qDot
> 0.000000  tau
> 0.000000  rhs: qDot
> 4.905000  rhs: qDDot
> Press any key to continue.


  * Press any key to continue
  * Convergence should be reached in around 24 iterations after a few seconds.
6. Run the least-squares problem
```
muscod_release pendulumLSQ
```
  * The screen will output information to the screen, nearly the same as before though the problem configuration has changed


> Dat file name            : DAT/pendulumLSQ.dat
> Problem Type             : Least-Squares
> Model file               : ../DAT/pendulum.lua
> LSQ target states file   : ../DAT/lsqDataToTrack.csv
> LSQ Reg. Weighting: 0.001
> 
> ----------------------------------------
> Numerical check of the model: 
>   Setting the model state and evaluating its generalized accelerations
> -1.570796 q
> 0.000000  qDot
> 0.000000  tau
> 0.000000  rhs: qDot
> 4.905000  rhs: qDDot


* As before, press a key to continue
* Convergence should be reached in around 103 iterations after 15 seconds or so.

# Code Layout of a MUSCOD-II + RBDL Problem

# Code Tour: Model

# Code Tour: Optimization Problem

# Notes 
* The csv file DAT/lsqDataToTrack.csv must have spaces after commas. LibreOffice Calc does not put these commas in by default: if you update this file you need to put them in by hand so that the SplineInterpolator.h class can load the file
