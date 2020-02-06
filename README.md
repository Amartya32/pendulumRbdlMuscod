# pendulumRbdlMuscod
A simple example of a minimization OCP and a least squares OCP where the model is made using RBDL and the problem is numerically solved using MUSCOD-II.

# Tested Configurations

* Ubuntu 18.04.4 LTS
* GNU Make 4.1
* Eigen 3.3.4
* RBDL 3.0.0, git commit: 4e36ba8a0fb9b68b45c25ad0fe40fc261bb3f351.
* MUSCOD-II, mercurial changeset:   186:326e1643e8c4

# Quick Start

1. Ensure that you have the following prerequisites, cloned, built, and installed:
  * C++ compiler
  * CMake
  * Eigen3 (install instructions appear on RBDL's page)
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

An optimal control problem in MUSCOD-II at a bare minimu requires a C++ file that defines the problem formulation (model stages, constraints, cost functions, etc. ) and a DAT file which tells MUSCOD-II how time, the state vector, the control vector, 

* SRC : this is the source directory
  * pendulum.cc 
    * The optimal control problem formulation is contained in here.
  * datfileutils.h
    * This contains functions to read information stored in the DAT file
  * PendulumModel.cpp and PendulumModel.h 
    * A wrapper for the RBDL pendulum model that takes care of setting the model up, and provides numerical functions to efficiently evaluate the state derivative, constraint residuals, and costs.
  * SplineInterpolator.h and string\_utils.h
    * These files contain functions to interpolate numerical data which is needed for the least-squares function since it is tracking sampled data.
* DAT  : MUSCOD-II looks in this directory for the DAT file
* RES : MUSCOD-II uses this directory to store results
* CMake : this directory contains a number of cmake files that search your computer to find Eigen3, RBDL, and Muscod
* build : the build directory

# Code Tour: pendulum.cc

1. Global Variables: search "Global Variables Block"
  * It is an ugly fact, but global variables are necessary so that all of the functions handles that MUSCOD-II calls can access the model and the various sizes everything. 
2. Problem Definition: search "void def_model(void)"
  * Search "Read user-defined DAT file variables": shows how to read user-defined variables from the DAT file 
  * Search "Test model using forward-dynamics": here the model's accelerations are computed given some hand coded data. The results are printed to the screen just to verify that the model has been loaded and is being evaluated correctly.
  * Search "Problem Definition": here the dimensions of the problem, the model stages, constraints and least-squares stages (if any) are defined. Read the MUSCOD-II manual carefully so that all of the functions that begin 'def\_' are clear.
  * Note that the size for each of the constraints (e.g. rdfcn\_icCount\_ne and rdfcn\_icCount\_n) and the function handle (rdfcn\_ic) come from global variables that are defined above
3. Dynamics: search "static void ffcn"
  * pendulumModel.updateState: when called copies over the variables sd, u, p
  * pendulumModel.calcForwardDynamicsRhs: when called evaluates the model's accelerations and then updates the res vector (which is a concatenation of qdot and qddot), which is the time derivative of sd (which is the concatenation of q and qdot).  
4. Constraints: in this problem there are constraints on the initial and final conditions
  * Search "static void rdfcn\_ic" and read the comments there.
    * Above this line the variables that define the number of equality constraints (rdfcn\_icCount\_ne) and the total number of constraints are defined (rdfcn\_icCount\_n).
    * In rdfcn\_ic the entries of res need to be updated. The size of res must match the variable that defines its size, in this case "rdfcn\_icCount\_". 
    * *Warning* if the array res is written to at an index beyond its actual size you may see weird and hard to debug problems: you will be writing to some other piece of memory, perhaps which is important to the problem. Use the 'assert' statements to catch these problems: when the code is run in debug mode the 'assert' statements will stop the program if they are not true; in a release model build these statements go away and do not slow the program down.
  * Search "static void rdfcn\_fc" and read the comments there. This constraint is very similar to the rdfcn\_ic function
5. Cost function: 
  * Search "static void lfcn"
  * The state of the model is updated: pendulumModel.updateState. 
  * The cost function's numerical value is evaluated: pendulumModel.calcCostTauSquared 
  * Scaled (this turns into a regularization term for a least-squares problem), and assigned to lval.

6. Least-Squares Cost function:
  * Search "void lsqfcn" and read the comments
  * Above this line the number of least squares terms "nlsq" is defined
  * The state of the model is updated: pendulumModel.updateState. 
  * The state of the least-squares interpolated solution: pendulum.updateStateLsq
  * Evaluate and assign least-squares residuals to res in two blocks q residuals, and qdot residuals.
  * Finally, note the use of the 'assert' statement to check that the number of res elements that have been written to is equal to the expected number of least-squares terms. Note: this statement is only checked when the model is run on a debug build.

7. Store data at every time-step for post-processing
  * Search "void mplo" and read the comments


# Code Tour: PendulumModel.h \& PendulumModel.cc

1. Model creation
2. 

# Notes 
* The csv file DAT/lsqDataToTrack.csv must have spaces after commas. LibreOffice Calc does not put these commas in by default: if you update this file you need to put them in by hand so that the SplineInterpolator.h class can load the file