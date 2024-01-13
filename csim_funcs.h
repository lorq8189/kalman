#ifndef CSIM_FUNCS_H
#define CSIM_FUNCS_H


#include <cstdio>
//#include <eigen3/Eigen/src/Core/Matrix.h>
//#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <random>
#include <string>
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

class Filter {

    public:
    
    void mainLoop(vector<double> measures, vector<double> controls, int n);

    Filter(int dimensions[], int inputMap[], vector<double> initial);











    

    int dm;

    int m;

    vector<RowVectorXd > x;  //estimate in present

    vector<RowVectorXd> xest; //state estimates for next step

    vector<RowVectorXd> Y; //state measurements

    MatrixXd H; //sensor matrix, adjust once you have specifics

    vector<MatrixXd> K; //Kalman gain

    MatrixXd A;  //state update

    MatrixXd B;  //Control input matrix

    vector<MatrixXd> P;  //covariance matrix

    MatrixXd R;  //Adds noise to stuff (bit more than that really)
    //covariance matrix of measurement noise (general measurement error (diagonals) + which ones get noisy when other ones get noisy (non-diagonals))

    MatrixXd Q;         //analogue of R but for controls, so B.

    //Note: both are to be set initially and tweaked during testing. They do not change like P does.
    //can we assume they are both diagonal matrices?
    
    vector<MatrixXd> U;  //represents controls

    MatrixXd I; //Identity matrix
    


    vector<MatrixXd> q;

    vector<double> r; //list of variances for noise


    vector<MatrixXd> PriorCov;

    vector<MatrixXd> BasisChange;

};



#endif