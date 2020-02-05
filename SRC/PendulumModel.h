#ifndef _PENDULUM_MODEL_H
#define _PENDULUM_MODEL_H



#include <rbdl/rbdl.h>

/**
  \brief An example of a typical RBDL model struct that is used when solving problems with MUSCOD-II
*/
class PendulumModel {
  public:
    /** Default constructor*/
    PendulumModel();
    /**
      @param luaModelFileName : path to the lua model
      @param verboseMessages: setting this to true will allow the function that
              reads the lua model file to print to the screen
    */
    PendulumModel(std::string& luaModelFileName,
                            bool verboseMessages);

    /**
      @param sd : a double array that contains the generalized positions for
                  this model followed by the generalized velocities. For the
                  pendulum model this vector contains the pendulum angle
                  followed by the angular velocity.
      @param u :  a double array that contains the controls for this model. For
                  the pendulum model this vector contains the applied torque.
      @param p :  a double array that contains the parameters being changed in
                  this model. For the pendulum model there are no parameters
                  being changed.

    */
    void updateState (
        const double *sd,
        const double *u,
        const double *p);

    /**
    This function computes the generalized accelerations of the model given
    a vector of generalized positions, velocities, and applied forces. In
    general this will be one of the most expensive functions to call in the
    model: only call it when needed.

    @param res : a double array that contains the generalized velocities for
                  this model followed by the generalized accelerations.
    */
    void calcForwardDynamicsRhs (double *res);

    unsigned int getDofCount();

  private:


    //This class maintains its own memory to save the
    //generalized positions, velocities, accelerations and
    //forces acting on the model.

    /**The RBDL model*/
    RigidBodyDynamics::Model model;
    /**The vector of generalized positions*/
    RigidBodyDynamics::Math::VectorNd q;
    /**The vector of generalized velocites*/
    RigidBodyDynamics::Math::VectorNd qdot;
    /**The vector of generalized accelerations*/
    RigidBodyDynamics::Math::VectorNd qddot;
    /**The vector of generalized forces*/
    RigidBodyDynamics::Math::VectorNd tau;

    /** After updateState has been called but before calcForewardDynamicsRhs
        has been called the vectors q, qdot, qddot, and tau are not dynamically
        consistent. In this case this flag is set to 'true'. This internal
        flag is not particularly useful in this simple model, but is quite
        useful for larger models to prevent both obvious mistakes and needlessly
        calling calcForwardDynamicsRhs*/
    bool dynamicsDirty;


    //This problem has no parameters that are being changed
    //RigidBodyDynamics::Math::VectorNd p;      //parameters
};

#endif



