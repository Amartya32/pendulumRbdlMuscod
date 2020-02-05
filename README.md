# pendulumRbdlMuscod
A simple example of a minimization OCP and a least squares OCP where the model is made using RBDL and the problem is numerically solved using MUSCOD-II.

# Code Layout of a MUSCOD-II + RBDL Problem

# Code Tour: Model

# Code Tour: Optimization Problem

# Notes 
* The csv file DAT/lsqDataToTrack.csv must have spaces after commas. LibreOffice Calc does not put these commas in by default: if you update this file you need to put them in by hand so that the SplineInterpolator.h class can load the file