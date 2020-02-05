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
    qdot    = VectorNd::Zero(model.dof_count);
    qddot   = VectorNd::Zero(model.dof_count);
    tau     = VectorNd::Zero(model.dof_count);

}

//==============================================================================
unsigned int PendulumModel::getDofCount(){
  return model.dof_count;
}

//==============================================================================
void PendulumModel::updateState (
    const double * sd,
    const double * u,
    const double *p) {

  dynamicsDirty = true;

  unsigned int idxTau   = 0;
  unsigned int idxQ     = 0;
  unsigned int idxQDot  = model.dof_count;

  //Extract q and qdot from sd
  for (unsigned int i = 0; i < model.dof_count; ++i) {
    q[i]    = sd[idxQ];
    ++idxQ;

    qdot[i] = sd[idxQDot];
    ++idxQDot;
  }

  //Zero out tau and qddot
  for (unsigned int i = 0; i < model.dof_count; ++i) {
    qddot[i] = 0.;
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

  ForwardDynamics(model,q,qdot,tau,qddot);
  dynamicsDirty = false;

  //Now go and update res, which is a concatenation of qdot, and qddot.
  unsigned int idxQDot   = 0;
  unsigned int idxQDDot  = model.dof_count;

  for(unsigned int i = 0; i< model.dof_count; ++i){
    res[idxQDot] = qdot[i];
    ++idxQDot;
    res[idxQDDot] = qddot[i];
    ++idxQDDot;
  }

}

