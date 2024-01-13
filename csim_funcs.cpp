#include <cstdio>
//#include <eigen3/Eigen/src/Core/Matrix.h>
//#include <eigen3/Eigen/src/Core/Matrix.h>

#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include "csim_funcs.h"
#include <eigen3/Eigen/Dense>
#include <vector>

using std::cin; using std::cout;
using std::ofstream;
using std::ifstream;
using std::string;
using std::cerr;
using std::vector;
using std::endl;

using Eigen::MatrixXd;
using Eigen::RowVectorXd;

Filter::Filter(int dimensions[], int inputMap[], vector<double> initial)
{


    dm = dimensions[0];
    m = dimensions[1];

    x[0] = RowVectorXd(dm);  //estimate in present
    for (int i=0; i<initial.size(); i++)
    {
        x[0][i] = initial[i];
        xest[0][i] = initial[i];
        Y[0][i] = initial[i];
    }
    
    MatrixXd A; //set state update matrix (A) here

    MatrixXd B; //set control input matrix (B) here

    MatrixXd H; //sensor matrix, set once you have specifics

    MatrixXd Q; //covariance matrix for B

    vector<MatrixXd> P;  //covariance matrix (do we set this initially?)

    vector<MatrixXd> R;  //Adds noise to stuff (not exactly)
    
    for (int i=0; i<dm; i++)
        I(i,i)=1;

    vector<MatrixXd> PriorCov;

    vector<MatrixXd> BasisChange;

}

void Filter::mainLoop(vector<double> measures, vector<double> controls, int n)
{

    std::default_random_engine rgen;
    std::normal_distribution<double> normal(0,1); //setting up random normal

    RowVectorXd q;


    x[n] = xest[n] + K[n]*(Y[n] - H*xest[n]); //change Y[n] to Y[n-1]??


    PriorCov[n] = (A*P[n]*A.transpose() + Q); //Prior covariance, using P(n),   |   placeholder, replace q*I
        

    //new Kalman Gain for next step 
    // K(n+1)
    K[n+1] = PriorCov[n] * H.transpose() * (H*PriorCov[n]*H.transpose() + R).inverse();


    //updating Covariance Matrix for next step (posterior estimation)
    // P(n+1)
    BasisChange[n] = I-K[n]*H;
    P[n+1] = BasisChange[n] * PriorCov[n] * BasisChange[n].transpose() + K[n]*R*(K[n]).transpose();

        
    for (int i=0; i<dm; i++)
        q[i] = normal(rgen)*Q[i,i]; //setting up q

    xest[n+1] = A*x[n] + B*U[n] + q; //+ q         q? whats q??? apparently q is just the output from an N(0, Q) (vector?)
        //estimates for next state, n+1 given n
        // Xest(n+1|n)
    
}


//OLD FILE, DEPRECATED
//NEW ONE IN OTHER DIRECTORY