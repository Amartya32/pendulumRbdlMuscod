/*
 M.Millard 2015/9/16

 This is part of my 'teach myself RBDL+PSOPT' by implementing
 an RBDL model (pendulum) and finding some optimal control for it
 (invert pendulum such that the integral of torque squared is 
 minimized)


*/

#include "PendulumModel.h"

#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <vector>
#include <limits>
#include <string>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//==============================================================================
PendulumModel::PendulumModel(){}
//==============================================================================
PendulumModel::PendulumModel(std::string& luaModelFileName,
                             std::string& lsqDataFileName,
                             int problemType,
                             bool verboseMessages)
{
    if(verboseMessages){
      cout << "----------------------------------------" << endl;
      cout << "Loading Lua Model" << endl;
    }
    bool modelIsLoaded = Addons::LuaModelReadFromFile(luaModelFileName.c_str(),
                                                      &model,
                                                      verboseMessages);
    if(verboseMessages){
      cout << "----------------------------------------" << endl << endl;
    }

    if (!modelIsLoaded) {
      cerr << "Error loading LuaModel: " << luaModelFileName << endl;
      abort();
    }

    q       = VectorNd::Zero(model.dof_count);
    qDot    = VectorNd::Zero(model.dof_count);
    qDDot   = VectorNd::Zero(model.dof_count);
    tau     = VectorNd::Zero(model.dof_count);

    assert(model.dof_count == 1);
    xLsq    = VectorNd::Zero(model.dof_count*2);
    qLsq    = VectorNd::Zero(model.dof_count);
    qDotLsq    = VectorNd::Zero(model.dof_count);

    lsqDataInterpolator.generateFromCSV(lsqDataFileName.c_str());

    switch(problemType){
      case 0:{
        isMinProblem = true;
        isLsqProblem = false;
      } break;
      case 1:{
        isMinProblem = false;
        isLsqProblem = true;
      } break;
      default:{
        std::cerr << " Error: problemType must be 0 or 1."
                  << " This is the value that was read: "
                  << problemType << std::endl;
        assert(0);
        abort();
      }
    }

}

//==============================================================================
unsigned int PendulumModel::getDofCount(){
  return model.dof_count;
}

//==============================================================================
bool PendulumModel::isMinimizationProblem(){
  return isMinProblem;
}
//==============================================================================
bool PendulumModel::isLeastSquaresProblem(){
  return isLsqProblem;
}

//==============================================================================
void PendulumModel::updateState (
    const double *sd,
    const double *u,
    const double *p) {

  dynamicsDirty = true;

  unsigned int idxTau   = 0;
  unsigned int idxQ     = 0;
  unsigned int idxQDot  = model.dof_count;

  //Extract q and qdot from sd
  for (unsigned int i = 0; i < model.dof_count; ++i) {
    q[i]    = sd[idxQ];
    ++idxQ;

    qDot[i] = sd[idxQDot];
    ++idxQDot;
  }

  //Zero out tau and qddot
  for (unsigned int i = 0; i < model.dof_count; ++i) {
    qDDot[i] = 0.;
    tau[i]   = 0.;
  }

  //Copy over the control signals into tau for this forward-dynamics
  //problem formulation.
  tau[0] = u[0];

  //We do nothing with p because there are no parameters being changed
  //in this model.

}


//==============================================================================
void PendulumModel::calcForwardDynamicsRhs(double *res){

  ForwardDynamics(model,q,qDot,tau,qDDot);
  dynamicsDirty = false;

  //Now go and update res, which is a concatenation of qdot, and qddot.
  unsigned int idxQDot   = 0;
  unsigned int idxQDDot  = model.dof_count;

  for(unsigned int i = 0; i< model.dof_count; ++i){
    res[idxQDot] = qDot[i];
    ++idxQDot;
    res[idxQDDot] = qDDot[i];
    ++idxQDDot;
  }

}

//==============================================================================
void PendulumModel::updateStateLsq (double *t){
  xLsq = lsqDataInterpolator.getValues(t[0]);
  qLsq[0]     = xLsq[0];
  qDotLsq[0]  = xLsq[1];
}
//==============================================================================
unsigned int PendulumModel::appendLsqErrorQ(double *res, unsigned int idx){
  res[idx] = q[0] - qLsq[0];
  ++idx;
  return idx;
}
//==============================================================================
unsigned int PendulumModel::appendLsqErrorQDot(double *res, unsigned int idx){
  res[idx] = qDot[0] - qDotLsq[0];
  ++idx;
  return idx;
}

//==============================================================================
double PendulumModel::calcCostTauSquared(double costWeighting){
  double cost = 0.;
  for(unsigned int i=0; i<tau.size();++i){
    cost += tau[i]*tau[i];
  }
  cost = cost*costWeighting;
  return cost;
}
//==============================================================================
unsigned int PendulumModel::
  appendEqIC(double *res, unsigned int idx)
{
  //Here the initial states have been hard coded. In general this
  //should be avoided, but for this simple example this rule has
  //been broken in hopes that this is easier to understand.
  res[idx] = q[0] - (-M_PI/2.0);
  ++idx;
  res[idx] = qDot[0] - 0;
  ++idx;
  return idx;
}
//==============================================================================
unsigned int PendulumModel::
  appendEqFC(double *res, unsigned int idx)
{
  res[idx] = q[0] - (M_PI);
  ++idx;
  res[idx] = qDot[0] - 0;
  ++idx;
  return idx;
}

unsigned int PendulumModel::
  appendEqFC_OverConstrained(double *res, unsigned int idx)
{
  res[idx] = q[0] - (M_PI);
  ++idx;
  res[idx] = qDot[0] - 0;
  ++idx;
  res[idx] = (qDot[0] - 0)*(q[0] - (M_PI));
  ++idx;
  return idx;
}
