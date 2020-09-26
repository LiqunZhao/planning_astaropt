//
// Created by lcr on 12/20/19.
//

#ifndef LANELET2_SMOOTHER_H
#define LANELET2_SMOOTHER_H
#include <iostream>
#include <vector>
#include "Array.hh"
#include <cmath>
#include "QuadProg++.hh"
class smoother {
public:
    quadprogpp::Matrix<double> Qacc;
    quadprogpp::Matrix<double> Qjerk;
    quadprogpp::Matrix<double> Qd;
    quadprogpp::Vector<double> Pacc;
    quadprogpp::Vector<double> Pjerk;
    quadprogpp::Matrix<double> Q;
    quadprogpp::Vector<double> P;
    quadprogpp::Matrix<double> Aeq, A;
    quadprogpp::Vector<double> beq, b;
    int length_;
    void setupQP(double* init, double* weight);
    void setupAb(quadprogpp::Matrix<double> Aeq_, quadprogpp::Matrix<double> A_,quadprogpp::Vector<double> beq_,quadprogpp::Vector<double> b_);
    double dt_;

    /// length is the number of ponit(x,y) ex. (1,2),(2,3)=> length=2;
    /// x is the output you define before use.
    /// init is the init state include x0 y0 vx0 vy0 ax0 ay0;
    /// weight is the weight of the cost function which include wacc wjerk wdis (we use wacc=0.5;wjerk=4;wdist=40;)
    /// point is the input point  {x1 y1 x2 y2 ...}
    void setting(int length, double* init, double* weight, double* point,double dt);
    void smoothing(quadprogpp::Vector<double>& x);
};


#endif //LANELET2_SMOOTHER_H
