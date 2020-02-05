#ifndef _PENDULUM_MODEL_H
#define _PENDULUM_MODEL_H



#include <rbdl/rbdl.h>

#include "SplineInterpolator.h"

/**
  \brief An example of a typical RBDL model struct that is used when solving problems with MUSCOD-II
*/
class PendulumModel {
  public:
    /** Default constructor*/
    PendulumModel();
    /**
      @param luaModelFileName : path to the lua model
      @param problemType: 0 for a minimization problem,
                          1 for a least-squares problem
      @param verboseMessages: setting this to true will allow the function that
              reads the lua model file to print to the screen
    */
    PendulumModel(std::string& luaModelFileName,
                  std::string& lsqDataFileName,
                  int problemType,
                  bool verboseMessages);

    //
    // Dynamics related functions
    //


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
    Evaluates the accelerations of the model. Important: ensure that
    updateState has been called prior to calling this function!

    @param res : a double array that contains the generalized velocities for
                  this model followed by the generalized accelerations.
    */
    void calcForwardDynamicsRhs (double *res);

    unsigned int getDofCount();

    //
    // Optimization Related functions
    //

    bool isMinimizationProblem();
    bool isLeastSquaresProblem();

    /**
      Updates the target state that the least-squares problem is trying to
      track for this specific instant in time.
    @param t: the current time which is required to estimate the state
              using the data sample and an interpolator
    */
    void updateStateLsq(double *t);

    /**
    Appends the value of the residuals for the positions tracked in the
    least squares problem starting at idx. Important: ensure that
    updateState and updateStateLsq has been called prior to calling this function!
    @param res: the array that contains all of the constraint errors
    @param idx: the index in res to assign the first constraint error
    @return the updated index value for the next block of constraints
    */
    unsigned int appendLsqErrorQ(double *res, unsigned int idx);
    const static int lsqCountQ = 1;

    /**
    Appends the value of the residuals for the positions tracked in the
    least squares problem starting at idx. Important: ensure that
    updateState and updateStateLsq has been called prior to calling this function!

    @param res: the array that contains all of the constraint errors
    @param idx: the index in res to assign the first constraint error
    @return the updated index value for the next block of constraints
    */
    unsigned int appendLsqErrorQDot( double *res, unsigned int idx);
    const static int lsqCountQDot = 1;



    /**
    Evaluates the weighted cost at this instant in time. Important: ensure that
    updateState has been called prior to calling this function!

    @param costWeighting: scale the cost by this value
    */
    double calcCostTauSquared(double costWeighting);


    /**
    Appends the value of the equality constraints for the initial conditions
    starting at idx. Important: ensure that
    updateState has been called prior to calling this function!
    @param res: the array that contains all of the constraint errors
    @param idx: the index in res to assign the first constraint error
    @return the updated index value for the next block of constraints
    */
    unsigned int appendEqIC(double *res, unsigned int idx);
    const static int eqCountIC = 2; //Number of entries for this constraint

    /**
    Appends the value of the equality constraints for the final conditions
    starting at idx. Important: ensure that
    updateState has been called prior to calling this function!

    @param res: the array that contains all of the constraint errors
    @param idx: the index in res to assign the first constraint error
    @return the updated index value for the next block of constraints
    */
    unsigned int appendEqFC(double *res, unsigned int idx);
    const static int eqCountFC = 2; //Number of entries for this constraint


  private:


    //This class maintains its own memory to save the
    //generalized positions, velocities, accelerations and
    //forces acting on the model.

    /**The RBDL model*/
    RigidBodyDynamics::Model model;
    /**The vector of generalized positions*/
    RigidBodyDynamics::Math::VectorNd q;
    /**The vector of generalized velocites*/
    RigidBodyDynamics::Math::VectorNd qDot;
    /**The vector of generalized accelerations*/
    RigidBodyDynamics::Math::VectorNd qDDot;
    /**The vector of generalized forces*/
    RigidBodyDynamics::Math::VectorNd tau;

    SplineInterpolator<RigidBodyDynamics::Math::VectorNd> lsqDataInterpolator;
    /**The interpolated data from the least-squares data file*/
    RigidBodyDynamics::Math::VectorNd xLsq;
    /**The current target q for the least-squares problem*/
    RigidBodyDynamics::Math::VectorNd qLsq;
    /**The current target qdot for the least-squares problem*/
    RigidBodyDynamics::Math::VectorNd qDotLsq;


    /** After updateState has been called but before calcForewardDynamicsRhs
        has been called the vectors q, qdot, qddot, and tau are not dynamically
        consistent. In this case this flag is set to 'true'. This internal
        flag is not particularly useful in this simple model, but is quite
        useful for larger models to prevent both obvious mistakes and needlessly
        calling calcForwardDynamicsRhs*/
    bool dynamicsDirty;

    bool isLsqProblem;
    bool isMinProblem;

    //This problem has no parameters that are being changed
    //RigidBodyDynamics::Math::VectorNd p;      //parameters
};

#endif



