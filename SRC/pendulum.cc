/*
@author M.Millard 2015

A toy problem just for learning:

    Single pendulum:
    ic: rest at 9 o'clock (gravity points at 6 o'clock)
    ec: rest at 12 o'clock 
    cost: integral of torque squared

State vector:
    x[0] = d/dt theta (rad/s)
    x[1] = theta (rad)

Controls
    u[0] = tau (Nm) 
 */





#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>    
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>


#include "PendulumModel.h"
#include "csvtools.h"

//Muscod related libraries need to access data and the dat file internal
//variables
#include "def_usrmod.hpp"
#include <DataCentral.hpp>
#include "datfileutils.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//An empty model struct
PendulumModel pendulumModel;

// datfileutils.h needs VectorNd and therefore RigidBodyDynamics::Math
//#include "datfileutils.h"

int nmos   = -1;  /* Number of phases (MOdel Stages) */
int np     = -1;  /* Number of parameters */
int nrc    =  0;  /* Number of coupled constraints */
int nrce   =  0;  /* Number of coupled equality constraints */

int nxd    = -1;  /* Number of differential states */
int nxa    =  0;  /* Number of algebraic states */
int nu     = -1;  /* Number of controls */
int npr    =  0;  /* Number of local parameters */


vector< vector < double > > posRecMatrix;
string meshupFileName = "RES/meshup_pendulum.csv";


vector< vector < double > > dataRecMatrix;
string dataFileName = "results.csv";

string modelFile            ;//="../model/MetaLuaModel2d.lua";
string datFileName          ;//="../DAT/MuscleDrivenWalker.dat";
string lsqTargetStateTrajectory ;//= "../expData/qIK.csv";
double lsqRegWeighting;
bool isMinProblem;
bool isLSQProblem;
/*
//No Mayer cost for this problem, so its not included
static void mfcn(double *ts, double *sd, double *sa,
double *p, double *pr, double *mval, long *dpnd, InfoPtr *info)
*/

static void lfcn(   double *t, double *sd,  double *sa,  double *u,
                   double *p, double *lval,double *rwh,  long *iwh, 
                                                     InfoPtr *info)
{
   double tau = u[0];
  *lval = tau*tau ;
}

static void ffcn(  double *t, double *sd, double *sa, double *u,
            double *p,  double *rhs,  double *rwh,  long *iwh, 
                                                InfoPtr *info)
{    
    
  pendulumModel.updateState(sd,u,p);
  pendulumModel.calcForwardDynamicsRhs(rhs);

}

static void ic(  double *ts, double *sd, double *sa,     double *u,
                double *p,  double *pr, double *res,    long *dpnd, 
                                                    InfoPtr *info)
{
    //This tells MUSCOD which partial derivatives to compute
    if (*dpnd) {
        *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
        return;
    }    


    res[0] = sd[0] - (-M_PI/2.0);
    res[1] = sd[1] - 0;
}


static void fc(  double *ts, double *sd, double *sa,     double *u,
                double *p,  double *pr, double *res,    long *dpnd, 
                                                    InfoPtr *info)
{
    //This tells MUSCOD which partial derivatives to compute
    if (*dpnd) {
        *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0);
        return;
    }    

    res[0] = sd[0] - M_PI;
    res[1] = sd[1] - 0;
}



//Called at every successful integrator step
void mplo(  double *t,                  //time
                double *sd,                 //differential state
                double *sa,                 //algebraic state
                double *u,                  //control
                double *p,                  //parameters
                double *rwh,                //real working array
                long *iwh,                  //integer working array
                InfoPtr *info) {            //info



    //Time to print the files.
    if(*t == 0.){

        bool append = false;

        if(posRecMatrix.size() > 1){        
            string emptyHeader = "";
            printMatrixToFile(  posRecMatrix, 
                                emptyHeader,
                                meshupFileName, 
                                append);
            posRecMatrix.clear();
           //posRecMatrix.resize(1);
        }   

        if(dataRecMatrix.size() > 1){            
            char buffer[500];
            snprintf(buffer, 500, "%s","time,");
            for(int i=0; i<nxd; ++i)
                snprintf(buffer + strlen(buffer), 500, " sd%i,", i);
            
            for(int i=0; i<nu; ++i)
                snprintf(buffer + strlen(buffer), 500," u%i,", i);

            string dataHeader = buffer;
            printMatrixToFile(  dataRecMatrix, 
                                dataHeader, 
                                dataFileName, 
                                append);


            dataRecMatrix.clear();
            //dataRecMatrix.resize(1);
        }


    }else{

        //accumulate the position matrix
        
        vector < double > posRecVector(1 + nxd/2);            
        int idx = 0;
        posRecVector[idx] = t[0];
        ++idx;
        for(int i=0; i<nxd/2; ++i){
            posRecVector[idx] = sd[i];
            ++idx;
        }
        posRecMatrix.push_back(posRecVector);

        //accumulate the data matrix
        
        vector < double > dataRecVector(1 + nxd + nu);
        //dataRecVector.resize(1 + nxd + nu);

        idx = 0;
        dataRecVector[idx] =  t[0];
        ++idx;
        for(int i=0; i<nxd; ++i){
            dataRecVector[idx] = sd[i];
            ++idx;
        }
        for(int i=0; i<nu; ++i){
            dataRecVector[idx] = u[i];
            ++idx;
        }        
        dataRecMatrix.push_back(dataRecVector);

    }


}

//Called at the beginning and end of every shooting interval
void mout
(
  long   *imos,      ///< index of model stage (I)
  long   *imsn,      ///< index of m.s. node on current model stage (I)
  double *ts,        ///< time at m.s. node (I)
  double *te,        ///< time at end of m.s. interval (I)
  double *sd,        ///< differential states at m.s. node (I)
  double *sa,        ///< algebraic states at m.s. node (I)
  double *u,         ///< controls at m.s. node (I)
  double *udot,      ///< control slopes at m.s. node (I)
  double *ue,        ///< controls at end of m.s. interval (I)
  double *uedot,     ///< control slopes at end of m.s. interval (I)
  double *p,         ///< global model parameters (I)
  double *pr,        ///< local i.p.c. parameters (I)
  double *ccxd,
  double *mul_ccxd,  ///< multipliers of continuity conditions (I)
#if defined(PRSQP) || defined(EXTPRSQP)
  double *ares,
  double *mul_ares,
#endif
  double *rd,
  double *mul_rd,    ///< multipliers of decoupled i.p.c. (I)
  double *rc,
  double *mul_rc,    ///< multipliers of coupled i.p.c. (I)
  double *obj,
  double *rwh,       ///< real work array (I)
  long   *iwh)        ///< integer work array (I)
{

      InfoPtr info(0, *imos, *imsn);
      mplo( ts, sd, sa, u, p, rwh, iwh, &info);


}






// Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void)
{

    std::stringstream datSs;
    datSs << "DAT/" << MCData->Options.datName << ".dat";
    datFileName = datSs.str();

    modelFile            = datfile_get_string(datFileName.c_str(),"modelFile");

    lsqTargetStateTrajectory = datfile_get_string(datFileName.c_str(),
                                "lsqProblemTargetStateTrajectoryFile");

    lsqRegWeighting      = datfile_get_double(datFileName.c_str(),
                                    "lsqRegularizationWeighting");

    int problemType = datfile_get_int(datFileName.c_str(),
                                      "problemType");

    switch(problemType){
      case 0:{
        isMinProblem = true;
        isLSQProblem = false;
      } break;
      case 1:{
        isMinProblem = false;
        isLSQProblem = true;
      } break;
      default:{
        std::cerr << " Error: problemType must be 0 or 1."
                  << " This is the value that was read: "
                  << problemType << std::endl;
        assert(0);
        abort();
      }
    }

    cout << endl;
    cout << "Dat file name            : " << datFileName << endl;
    if(isMinProblem){
      cout<<"Problem Type             : Minimization" << endl;
    }
    if(isLSQProblem){
      cout<<"Problem Type             : Least-Squares" << endl;
    }
    cout << "Model file               : " << modelFile << endl;
    cout << "LSQ target states file   : " << lsqTargetStateTrajectory << endl;
    cout << "LSQ Reg. Weighting: "    << lsqRegWeighting << endl;
    cout << endl;

    pendulumModel = PendulumModel(modelFile, true);
  
    vector< double > sdTest(2);
    vector< double > uTest(1);
    vector< double > rhsTest(2);
    vector< double > pTest(1);

    //Test the model: Put it in a position where you know what acceleration
    //                you should get. Numerically evalute this if possible, or
    //                if the model is very simple (as in this case) just print
    //                it to screen.
    assert(pendulumModel.getDofCount() == 1);
    printf("----------------------------------------\n");
    printf("Numerical check of the model: \n");
    printf("  Setting the model state and evaluating its"
           " generalized accelerations\n");
    sdTest[0] = -M_PI/2; //q
    sdTest[1] = 0;       //qdot
    uTest[0]  = 0;       //tau
    rhsTest[0] = 0;
    rhsTest[1] = 0;

    pendulumModel.updateState(sdTest.data(), uTest.data(), pTest.data());
    printf("%f\tq\n",     sdTest[0]);
    printf("%f\tqDot\n",  sdTest[1]);
    printf("%f\ttau\n",   uTest[0]);

    pendulumModel.calcForwardDynamicsRhs(rhsTest.data());
    printf("%f\trhs: qDot\n",   rhsTest[0]);
    printf("%f\trhs: qDDot\n",  rhsTest[1]);

    printf("Press any key to continue.\n");    
    cin.get();

    nmos    = 1;
    np      = 0;
    nrc     = 0;
    nrce    = 0;
    nxd     = pendulumModel.getDofCount() * 2;
    nxa     = 0;
    nu      = pendulumModel.getDofCount();

    printf( "nmos: %i, np: %i, nrc: %i, nrce: %i nxd: %i "
            "nxa: %i nu: %i\n",
            nmos, np, nrc, nrce, nxd, nxa, nu);

    //Define problem dimensions
    def_mdims(nmos,np,nrc,nrce);

    def_mstage( 0,      //Stage index
                nxd,    //number of continuous states
                nxa,    //number of algebraic states
                nu,     //number of controls
                NULL,   //Meyer function
                lfcn,   //Lagrange function
                0,      //DAESOL specific jacmlo
                0,      //DAESOL specific jacmup
                0,      //DAESOL specific astruc
                NULL,   //DAESOL specific afcn
                ffcn,   //rhs of the state equation
                NULL,   //gfcn rhs of the algebraic equation
                NULL,   //rwh : real work array-to pass common stage data
                NULL ); //iwh : integer work array-to pass common stage data

    def_mpc(0,          //Stage index
            "s",        //scope
            npr,        //number of local parameters
            2,          //Dimension of the decoupled residual
            2,          //Number of zeros
            ic,         //Decoupled constraint function handle
            NULL);      //Coupled constraint function handle;


    def_mpc(0,          //Stage index
            "e",        //scope
            npr,        //number of local parameters
            2,          //Dimension of the decoupled residual
            2,          //Number of zeros
            fc,         //Decoupled constraint function handle
            NULL);      //Coupled constraint function handle;
          

    def_mio (   NULL ,     //minp
                mout,      //mout
                mplo); //mplo


    
    


}



