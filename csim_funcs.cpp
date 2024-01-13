#include <cstdio>
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

    x[0] = VectorXd(dm);  //estimate in present
    for (int i=0; i<initial.size(); i++)
    {
        x[0][i] = initial[i];
        xest[0][i] = initial[i];
        Y[0][i] = initial[i];
    }

    
    MatrixXd A; //set state update matrix (A) here

    MatrixXd B; //set control input matrix (B) here

    MatrixXd H; //sensor matrix, set once you have specifics
    //H turns an x estimate format (dm x 1) to a y format (m x 1), so H is (m x dm)
    //takes a state prediction and turns it into what the sensors should get if that were the true state

    MatrixXd Q; //covariance matrix for B (quoting aaren)

    vector<MatrixXd> P;  //covariance matrix (we dont set this initially?)

    vector<MatrixXd> R;  //covariance matrix for noise of sensors

    I = MatrixXd::Identity(dm, dm);

    vector<MatrixXd> PriorCov;

    vector<MatrixXd> BasisChange;

    //for above two matrices, notation BasisChange[n] means the BasisChange matrix USING data from step n, and the matrix is used in step n+1

}


//for first loop, n = 1, we need K[0] P[0] x[0] U[0] 

void Filter::mainLoop(int n)
{
    //for iteration n: predicts xest[n], then generates values for x[n], K[n], P[n] using n-1 data and xest[n]
    //calculates x[n] with xest[n], Previous K, and current Y

    //setup U and Y, controls and measurements



    std::default_random_engine rgen;
    std::normal_distribution<double> normal(0,1); //setting up random normal
    VectorXd q;

    for (int i=0; i<dm; i++)
        q[i] = normal(rgen)*Q(i,i); //setting up q, might need to change later if assuming Q is non-diagonal (probably not actually)

    xest[n] = A*x[n-1] + B*U[n-1] + q; //        q? whats q??? apparently q is just the output from an N(0, Q) (vector?)
        //estimates for this state, n given n-1
        // possibly change to being Xest(n|n-1) and at the beginning of the loop instead (this change was made)


    x[n] = xest[n] + K[n-1]*(Y[n] - H*xest[n]); //change Y[n-1] to Y[n]? (this change was made)


    PriorCov[n-1] = (A*P[n-1]*A.transpose() + Q); //Prior covariance, using P(n-1)
        
    //new Kalman Gain for next step 
    // K(n+1)
    K[n] = PriorCov[n-1] * H.transpose() * (H*PriorCov[n-1]*H.transpose() + R).inverse();


    //updating Covariance Matrix for next step (posterior estimation)
    // P(n+1)
    BasisChange[n-1] = I-K[n-1]*H; 
    P[n] = BasisChange[n-1] * PriorCov[n-1] * BasisChange[n-1].transpose() + K[n-1]*R*(K[n-1]).transpose();

    
}


// product takes vertical of first and horizontal of second, 
// horizontal of first must equal vertical of second


//OLD FILE, DEPRECATED
//NEW ONE IN OTHER DIRECTORY

//void Filter::mainLoop(VectorXd Yn, VectorXd Un, int n)