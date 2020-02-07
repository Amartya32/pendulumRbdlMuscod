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

2. Ensure that you know how to use basic debugging tools: you can start a MUSCOD-II problem in debug mode, use break points, and interpret information from a stack trace. Optimal control problems are very challenging to develop: if you do not know how use these basic debugging tools you spend an enormous amount of time fixing your code. If you don't know how to do these things learn now. If you want to use an nice developer environment VS Code, or QtCreator are good choices. If you want to do everything from the command line you'll have to learn gdb. Information for all of these tools is available online.

3. Make a build folder within pendulumRbdlMuscod

4. Run cmake (or ccmake) from the 'build' folder. You will be asked to set CUSTOM\_RBDL\_PATH, which is the path to the install folder of the version of RBDL that you would like to use. If you have a system-wide installation of RBDL this folder is '/usr/local'

5. From a terminal in the build directory call 
  ```
    make
  ```    

6. Run the minimization problem
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
7. Run the least-squares problem
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
* Convergence should be reached in around 4 iterations after 5 seconds or so.

# Code Layout of a MUSCOD-II + RBDL Problem

An optimal control problem in MUSCOD-II at a bare minimu requires a C++ file that defines the problem formulation (model stages, constraints, cost functions, etc. ) and a DAT file which tells MUSCOD-II how time, the state vector, the control vector, 

* SRC : this is the source directory
  * pendulum.cc 
    * The optimal control problem formulation is contained in here.
  * PendulumModel.cpp and PendulumModel.h 
    * A wrapper for the RBDL pendulum model that takes care of setting the model up, and provides numerical functions to efficiently evaluate the state derivative, constraint residuals, and costs.
  * SplineInterpolator.h and string\_utils.h
    * These files contain functions to interpolate numerical data which is needed for the least-squares function since it is tracking sampled data.
  * datfileutils.h
    * This contains functions to read information stored in the DAT file    

* DAT  : MUSCOD-II looks in this directory for the DAT file
  * pendulum.lua  : the Lua description of the RBDL model   
  * unit\_cube.obj : a mesh used to animate the pendulum
  * pendulumMIN.dat : the DAT file for the minimization problem
  * pendulumLSQ.dat : the DAT file for the least-squares problem  
  * lsqDataToTrack.csv : the data that contains a time series of the pendulum angle and angular velocity which is used in the least-squares problem to compute the least-squares residuals. Note that the csv file DAT/lsqDataToTrack.csv must have spaces after commas otherwise SplineInterpolator.h's functions will throw an error. LibreOffice Calc does not put these commas in by default: if you update this file you need to put them in by hand.

* RES : MUSCOD-II uses this directory to store results
  * meshupPendulum.csv : a time series of the model's positions which is used to animate the model using Meshup/rbdl-toolkit.
  * simulationDataPendulum.csv: a time series of the model's positions, velocities, and controls. All data that is needed for post-processing can be put in this file. To make it easy to access a specific column we recommend giving each column a human-readable name (as we have here) which allows you to access each column by name later.
  * MUSCOD-II output: bin, log and txt files. The bin file contains all of the numerical values MUSCOD-II needs to re-start the problem (done by using the ```-w``` flag ```muscod_release -w ...```): note if the size of any of the problem dimensions change you cannot start MUSCOD-II using a warm start. The log file contains all of the text that is printed to the terminal when MUSCOD-II is running. The txt file contains the time, state, control, and parameter values at the beginning of each shooting node. You can extract these values (write a script to do this) and use them in the DAT file if you'd like to begin
  * MUSCOD-II optional output: By setting "options\_output\_ps" to "1" in the DAT file MUSCOD-II will save the state and control plots to post-script files in this folder.

* CMake : this directory contains a number of cmake files that search your computer to find Eigen3, RBDL, and Muscod

* build : the build directory
  * CMakeCache.txt : if you need to re-run CMake and want to force it to start fresh delete this file. If you have never seen this file before open it in a text editor and have a look at it: it just stores the value of all of the variables CMake requires to build this project. For example if you search for "CUSTOM\_RBDL\_PATH" you'll find the folder location you entered in here.
  * libpendulum.so : the library the result of compiling SRC/pendulum.cc and its dependencies. The name of this library must be listed in the DAT file after the variable "libmodel". Open pendulumLSQ.dat and pendulumMIN.dat now, and search for "libmodel": you should see libpendulum listed right after it.


# Code Tour: pendulum.cc

1. Global Variables: search "Global Variables Block"
  * It is an ugly fact, but global variables are necessary so that all of the functions handles that MUSCOD-II calls can access the model and the various sizes everything. 

2. Problem Definition: search ```void def_model(void)```
  * Search "Read user-defined DAT file variables": shows how to read user-defined variables from the DAT file 
  * Search "Test model using forward-dynamics": here the model's accelerations are computed given some hand coded data. The results are printed to the screen just to verify that the model has been loaded and is being evaluated correctly.
  * Search "Problem Definition": here the dimensions of the problem, the model stages, constraints and least-squares stages (if any) are defined. Read the MUSCOD-II manual carefully so that all of the functions that begin ```def_``` are clear.
  * Note that the size for each of the constraints (e.g. ```rdfcn_icCount_ne``` and ```rdfcn_icCount_n```) and the function handle (```rdfcn_ic```) come from global variables that are defined above

3. Dynamics: search "static void ffcn"
  * pendulumModel.updateState: when called copies over the variables sd, u, p
  * pendulumModel.calcForwardDynamicsRhs: when called evaluates the model's accelerations and then updates the res vector (which is a concatenation of qdot and qddot), which is the time derivative of sd (which is the concatenation of q and qdot).  

4. Constraints: in this problem there are constraints on the initial and final conditions
  * Search ```static void rdfcn_ic``` and read the comments there.
    * Above this line the variables that define the number of equality constraints (```rdfcn_icCount_ne```) and the total number of constraints are defined (```rdfcn_icCount_n```).
    * In ```rdfcn_ic``` the entries of res need to be updated. The size of res must match the variable that defines its size, in this case ```rdfcn_icCount_```. 
    * *Warning* if the array res is written to at an index beyond its actual size you may see weird and hard to debug problems: you will be writing to some other piece of memory, perhaps which is important to the problem. Use the ```assert``` statements to catch these problems: when the code is run in debug mode the ```assert``` statements will stop the program if they are not true; in a release model build these statements go away and do not slow the program down.
  * Search ```static void rdfcn_fc``` and read the comments there. This constraint is very similar to the rdfcn\_ic function

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
  * Finally, note the use of the ```assert``` statement to check that the number of res elements that have been written to is equal to the expected number of least-squares terms. Note: this statement is only checked when the model is run on a debug build.

7. Store data at every time-step and when the current SQP iteration is complete write the data to file
  * Search "void mplo" and read the comments


# Code Tour: PendulumModel.h & PendulumModel.cc

This class provides three kinds of functions: functions with manage the data of the class, functions which evaluate the dynamics of the model, and functions which support optimization. This class has been designed following a few style points to reduce errors:

* All of the model's memory is private and can only be accessed by functions.
* All of the functions to numerically evaluate optimization related quantities (the cost function, constraint residuals, least-squares residuals) are contained in the model class

The reasons for these two deliberate decisions are as follows:

* To reduce the amount of code in the problem formulation (e.g. in pendulum.cc)
* To prevent the spread of bugs by having a single implementation for every numerical function
* To encourage the re-use of code
* To prevent the temptation to manipulate RBDL's model class directly: few people do this correctly.

For a more complicated problem it is typical to have large blocks of related constraints that are re-used throughout the problem (e.g. foot-ground forces are always up). If these constraints are implemented in the model then they can be used throughout the problem consistently. If there is a bug in these constraints then there is only one place to fix it. 

It is far more common to copy the code that implements a specific constraint and to embed it throughout the the problem formulation code. This is asking for trouble. First, the problem formulation code will quickly become huge which makes it hard to work with. Second, if you find a bug in a function that is used throughout the code, then you have to correct it in many different places. Few people can manage to do this correctly on a medium to large problem. Unfortunately even if you succeed, you still fail with this approach: nobody else will be able to use this code later. I strongly recommend to implement the code that evaluates the cost function, constraint residuals, and least-squares residuals in the model class. I also recommend breaking up these functions into logical blocks so that you can re-use the blocks.


## Data Management Functions


1. Class constructors: ```PendulumModel()``` and ```PendulumModel(std::string& luaModelFileName, std::string& lsqDataFileName, int problemType, bool verboseMessages)```: go read the doxygen in the header file and have a look at the implementation in the cc file.

2. Local memory cache update: (```updateState(const double *sd, const double *u, const double *p)```, ```updateStateLSQ(double *t)```). All of the numerical functions in the model manipulate the model state, controls, and parameters. Since copying over memory is expensive for large models we have separated the updating of the state, control, and parameters of the model from the numerical functions. This means that your problem will run fast, but it also means that in the optimal control problem file (pendulum.cc) you must take care to call ```updateState``` or ```updateStateLSQ``` once prior to evaluating any numerical functions.

3. Model Properties: There are large number of properties that you can get out of the model, here we just return one ```getDofCount()```. Read the documentation 

## Dynamics Functions

1. Forward Dynamics: For this simple problem we have just one: ```calcForwardDynamicsRhs(double *res)```. This function evaluates the generalized accelerations of the model and updates the derivative of the state vector. Look at the code and comments in both PendulumModel.h and PendulumModel.cc.

## Optimization Functions

Examine the comments for each of these functions in PendulumModel.h, and then look at the implementation in PendulumModel.cc
 
1. Cost Function: ```calcCostTauSquared(double costWeighting)```. 

2. Constraint Function Residuals: ```appendEqIC(double *res, unsigned int idx)``` and ```appendEqFC(double *res, unsigned int idx)```. These functions are designed to write a block of values to the double array of constraint residuals starting at idx. So that the next block of residuals can be easily accessed, these functions pass out the index in res which follows the block. This extra book keeping effort has been made because the residuals are stored in a double array: if you mistakenly write beyond the final element of the array you will be writing to some other piece of memory. This is a very difficult bug to find because the code will run, the problem may even converge, but it will not be correct!

3. Least-Squares Residuals: ```appendLsqErrorQ(double *res, unsigned int idx)``` and ```appendLsqErrorQDot( double *res, unsigned int idx)```. Here the same structure is used for the constraint function residuals has been employed to prevent careless mistakes of indexing res beyond its allocated limit.


# Common Errors

Here we will cover the most common errors encountered by new users to MUSCOD-II

## DAT File Errors

The dimensions of the problem that are passed into the function ```def_mdims``` must be consistent with the number of entries for the initial conditions, minimal values, maximal values, and scaling applied to the states (sd, sd\_min, sd\_max, sd\_sca), the control vector (u, u\_min, u\_max, u\_sca), the constraint residual scaling (rd\_sca), and the parameter values (there are none in the simple pendulum problem). If there is an inconsistency MUSCOD-II will fail and print a message to the screen that will give you a hint of where the problem is. You can see this yourself:

1. Comment out an initial state in pendulumMIN.dat: replace "1: 0.000000" below "sd(0,S)" with "#1: 0.000000". 

2. Run MUSCOD-II. You should see a message that looks like this:

> LIBLAC run-time error:
> fscan_vec(): read error at element i = 1
> *** WARNING ***
> READ_VECTOR(): read error in vector after keyword "sd(\*,S)"
> ERROR #-1 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/INOUT/InOutDat.cpp':1118, function 'LoadIdentifiers':
>   Data I/O failed: ( No detailed problem information specified )
>   Read errors while loading identifiers.
> ERROR #-4 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/INOUT/InOutDat.cpp':247, function 'LoadFromDatFile':
>   Data I/O failed: Reading from device failed.
> ERROR #-4 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/MODEL/model.cpp':85, function 'InitializeModelFromDatFile':
>   Data I/O failed: Reading from device failed.

3. Read the error message carefully: within the first 4 lines that is enough information to tell you where the error is in the DAT file.

4. Be sure to restore the DAT file to its working condition before moving on.


## Residual Vector Indexing Errors

This class of error applies to any function in MUSCOD-II which requires you to update a double vector. This applies to any function that evaluates a constraint residual, or a least-squares residual. Here we will go through one example in detail:

1. Look at the functions ```rdfcn\_ic```: The main output of this function is updating the numerical values of the constraint residuals that are contained in the double array ```res```.

2. The size of ```res``` is defined when the command ```def_mpc``` is called for this constraint in the function ```def_model```:

> def_mpc(0,                //Stage index
>            "s",              //scope
>            npr,              //number of local parameters
>            rdfcn_icCount_n,  //Dimension of the decoupled residual
>            rdfcn_icCount_ne, //Number of zeros
>            rdfcn_ic,         //Decoupled constraint function handle
>            NULL); 

3. The constraint residuals must be written to res starting with the equality constraints (here set by variable ```rdfcn_icCount_ne```) followed by the inequality constraints (here ```rdfcn_icCount_n```-```rdfcn_icCount_ne```) 

4. There are no safe guards in place to prevent you from writing to an element of ```res``` that is larger than its defined size. If this happens you will be overwriting memory that is supposed to be used for something else: unpredictable behavior will result. To prevent this problem from happening ```assert``` statements have been added to check the size of the variable ```idx``` to make sure that it is consistent with the variables that define the size of ```res```.

5. To see for yourself what happens when things go wrong we can add an indexing error to a working problem:
  * In ```rdfcn_ic``` set the starting value of idx to 1: this will write the final constraint beyond the size of ```res```.
  * Build the problem in debug mode and run it. What happens? The program should stop and give you an informative error message similar to this:

> muscod_release: /home/mjhmilla/dev/projectsSmall/pendulumRbdlMuscod/SRC/pendulum.cc:168: void rdfcn_ic(double\*, double\*, double\*, double\*, double\*, double\*, double\*, long int\*, InfoPtr\*): Assertion idx == rdfcn_icCount_ne failed.
>  Aborted (core dumped)

  * Build the problem in release mode: this will remove the ```assert``` statements. And re-run the problem using ```muscod_release -i5 pendulumMIN```. On my machine the problem runs for 2 SQP iterations and then I get the error below. You may get a different error, or no error at all. All of these scenarios are possible. Unless you add the code to test the ```res``` vector is being correctly updated you have no way of knowing what is causing this kind of error. 

> ERROR #-959 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/QPS/qps_qpopt_new.cpp':474, function 'qpsSolve':
>   Input parameter is invalid
> ERROR #-996 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/COND/cond_stable.cpp':1298, function 'csolve_qp_solve':
>   Solution of condensed QP failed
> ERROR #-990 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/SOLVE/solve_slse_common.cpp':378, function 'solve_subp':
>   Solution of current sub QP failed
> ERROR #-974 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/MSSQP/mssqp_standard.cpp':370, function 'mssqpFeedback':
>   Step determination failed (in QP or line search)
> 
> ****  terminated due to an error  ****

6. Be sure to restore the code to its working condition before moving on.
  
## Problem Formulation Errors

There are many different kinds of problem formulation errors, here we will consider two:

1. The cost function is not being properly assigned. To simulate this error:
  * Find the function ```static void lfcn```
  * Comment out the line ```*lval = pendulumModel.calcCostTauSquared(scaling) ;``` and add this line below it ```*lval = 0.;```: clearly does not have a minimum, nor is it related in anyway to the system.
  * Build the problem in release mode
  * Run it using ```muscod_release pendulumMIN```
  * What happens? It converges in 4 iterations!!!
  * Anytime you see something converging very fast, you should be very suspicious and have a look at the code defining the cost function.

2. The problem is over constrained. This is one of the hardest problems to diagnose and fix because there aren't any tools to help you identify this as a problem aside from your own brain. You have to use your own understanding of the problem, hypothesize where the redundant constraint might exist, implement an update, and run the problem again. If the problem runs then you've successfully diagnosed and fixed it. If not, the problem lies elsewhere. To simulate this type of error:
  * Go to function ```static void rdfcn_fc```
  * Above this function comment out the line ```static int rdfcn_fcCount_ne  = pendulumModel.eqCountFC;```
  * Uncomment ```static int rdfcn_fcCount_ne  = pendulumModel.eqCountFC_OverConstrained;```
  * In the function comment out the line ```idx = pendulumModel.appendEqFC(res,idx);```
  * Uncomment ```idx = pendulumModel.appendEqFC_OverConstrained(res,idx);```
  * Open PendulumModel.cc and look at the function ```appendEqFC_OverConstrained```: the third constraint is the product of the two residuals. By definition this makes the problem over-constrained.
  * Compile the problem
  * Open the DAT file pendulumMIN.dat and set variable ```options_qp_relax``` to ```1.0```
  * Run the problem ```muscod_release pendulumMIN```
  * What do you see? The error message from MUSCOD-II is actually quite helpful:

> ERROR #-965 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/QPS/qps_qpopt_new.cpp':593, function 'qpsSolve':
>   Infeasible constraint set
> ERROR #-996 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/COND/cond_stable.cpp':1298, function 'csolve_qp_solve':
>   Solution of condensed QP failed
> ERROR #-991 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/SOLVE/solve_slse_common.cpp':277, function 'solve_minp':
>   Solution of minimum norm QP failed
> ERROR #-932 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/HESS/hess_update.cpp':54, function 'initializeHessian':
>   Initial estimate of Hessian approximation failed
> ERROR #-972 in '/home/mjhmilla/dev/MUSCOD_SUITE/MC2/Src/MSSQP/mssqp_standard.cpp':200, function 'mssqpPrepare':
>   Calculation of scaled identity Hessian approximation failed

  * If you see the error with the text "Infeasible constraint set" you take a close look at your constraints and see if there are any redundant constraints. 
  * Unfortunately the resulting error is not always so clear: if the model is large enough the solution process might get close to convergence, but will never actually converge. If you see this behavior you should start looking at your constraints.
  * This problem can also be hidden by changing some settings in the DAT file:  open the DAT file pendulumMIN.dat and set variable ```options_qp_relax``` to ```1.1```
  * Re-run the problem: on my machine it converges. When you see  ```options_qp_relax``` set to values greater than ```1.0``` you should be a bit suspicious that perhaps this setting is masking a redundant constraint problem.

## Model Related Problems

1. Numerical stiffness: optimal control problem solve nicely when the system dynamics are well approximated by simple functions within each time discretization. This may not be true if the model and/or problem is numerically stiff. To know if the model is well suited to being part of an optimal control problem it is valuable to know the eigen-frequencies of the model across all of the problems different stages: constraints/contact forces can change the eigen frequencies. These can be identified by numerically by applying an impulse to the model and looking at the power spectrum of the model's response. Intuition can also be used: mechanical systems with a wide range of masses, stiff springs, or very strong actuators applied to light bodies will cause problems. These problems can be fixed by removing the stiff element (if it is not necessary), or reformulating the problem: a forward-dynamics problem can be replaced with an inverse-dynamics problem; spring forces that are explicitly applied to the model can be implicitly applied instead. The problem reformulations just mentioned are not trivial and are far beyond the scope of this document to describe: but now you know what to search for. 

2. Poor Initial Conditions: if the initial values used for the shooting nodes are too far away from the solution MUSCOD-II may have convergence problems. In this case it pays to invest some time to synthetically create a better initial solution and then to set the appropriate variables in the DAT file.

