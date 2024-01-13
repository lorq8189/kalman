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



int main(int argc, char** argv ) {


    ifstream in;
    in.open(argv[1]);
    ofstream out;
    out.open("values.txt"); //change to csv later on
    ofstream preout;
    preout.open("predictions.txt");

    //the matwix
    //set of inputs: 0-2: all positions 3-8: all velocities and accelerations, same pattern
    //9-
    int dm;//dimension count, can change for later

    int m;
    
    int outc; //output count

    int inlength;

    //vector<MatrixXd> M(dm);

    // MatrixXd sample(dm, 1);

    // MatrixXd sample2(dm, 1);


    // vector<MatrixXd> x(dm);  //estimate in present

    // vector<MatrixXd> xest(dm, 1); //state estimates for next step

    // vector<MatrixXd> Y(m, 1); //state measurements

    // MatrixXd H(dm, dm); //sensor matrix, adjust once you have specifics

    // vector<MatrixXd> K(dm, dm); //Kalman gain

    // MatrixXd A(dm,dm);  //state update

    // MatrixXd B(dm,dm);  //Control input matrix

    // vector<MatrixXd> P(dm,dm);  //covariance matrix

    // vector<MatrixXd> R(dm,dm);  //Adds noise to stuff
    
    // vector<MatrixXd> U(dm, 1);  //represents controls

    // MatrixXd I(dm, dm); //Identity matrix

    // vector<MatrixXd> q(dm, 1);

    // vector<double> r; //list of variances for noise


    // vector<MatrixXd> PriorCov(dm, dm);

    // vector<MatrixXd> BasisChange(dm, dm);







    std::default_random_engine rgen;
    std::normal_distribution<double> normal(0,1); //setting up random normal

    out << m << std::endl;

    int a[] = {18, 8};
    int b[] = {0};
    vector<double> c;

    Filter f_nep(a, b, c);

    int n = 1;
    string line;
    while (std::getline(in, line)) {
        std::istringstream iss(line);

        string word;
        vector<double> measure;
        while(iss >> word)          //measurements into input vector
        {
            stod(word);
            measure.push_back(stod(word));
        }
        //turn measure into Y here once you have specifics

        //estimates for current state given measurements and past estimation
        //X(n|n) given X(n|n-1)
        // 


        f_nep.mainLoop(c, c, n);

        out << "  Estimates for time " << ++n << ":   ";
        for (int i=0; i<dm; i++)
            //out << x[n] << ", ";//write estimates to file here
        out << endl;

        preout << " Predictions for time " << n << ":   ";
        for (int i=0; i<dm; i++)
            //preout << xest[n] << ", ";//write predictions to file here
        preout << endl;
    }


    return 0;
}

//x[n] = xest[n] + K[n]*(Y[n] - H*xest[n]); //change Y[n] to Y[n-1]??


        // PriorCov[n] = (A*P[n]*A.transpose() + q); //Prior covariance, using P(n)
        // //new Kalman Gain for next step (whats R?)
        // // K(n+1)

        // for (int i=0; i<dm; i++)
        //     R[i] = normal(rgen)*r[i];

        // K[n+1] = PriorCov[n] * H.transpose() * (H*PriorCov[n]*H.transpose() + R).inverse();


        // //updating Covariance Matrix for next step (posterior estimation)
        // // P(n+1)
        // BasisChange[n] = I-K[n]*H;
        // P[n+1] = BasisChange[n] * PriorCov[n] * BasisChange[n].transpose() + K[n]*R[n]*(K[n]).transpose();

        
        // for (int i=0; i<18; i++)
        //     q[n][i]= normal(rgen)*Q[i,i]; //setting up q

        // xest[n+1] = A*x[n] + B*U[n] + q; //+ q         q? whats q??? apparently q is just the output from an N(0, Q)
        // //estimates for next state, n+1 given n
        // // Xest(n+1|n)