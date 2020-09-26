//
// Created by lcr on 12/20/19.
//

#include "smoother.h"

void smoother::setupQP(double* init, double* weight){
    double xinit = init[0];
    double yinit = init[1];
    double vxinit = init[2];
    double vyinit = init[3];
    double axinit = init[4];
    double ayinit = init[5];
    double wacc = weight[0];
    double wjerk = weight[1];
    double wdist = weight[2];
    ///setup length

    Qacc.resize(0,2*length_,2*length_);
    Qjerk.resize(0,2*length_,2*length_);
    Qd.resize(0,2*length_,2*length_);

    //Aeq.resize(0,2*length_,2*length_);
    //A.resize(0,2*length_,2*length_);

    Pacc.resize(0,2*length_);
    Pjerk.resize(0,2*length_);

    //beq.resize(0,2*length_);
    //b.resize(0,2*length_);

    ///setup Qacc

    for (int i=0;i<2*length_-4;i++ ){
        Qacc.s(6,i,i);
        Qacc.s(-4,i,i+2);
        Qacc.s(1,i,i+4);
    }
    Qacc.s(5,2*length_-4,2*length_-4);
    Qacc.s(-2,2*length_-4,2*length_-2);
    Qacc.s(-2,2*length_-2,2*length_-4);

    Qacc.s(5,2*length_-3,2*length_-3);
    Qacc.s(-2,2*length_-3,2*length_-1);
    Qacc.s(-2,2*length_-1,2*length_-3);

    Qacc.s(1,2*length_-2,2*length_-2);
    Qacc.s(1,2*length_-1,2*length_-1);
    for(int i=0;i<2*length_-4;i++){
        for(int j=i;j<i+5;j++){
            Qacc.s(Qacc[i][j],j,i);
        }
    }
    //std::cout<<Qacc<<std::endl;
    Qacc=(1.0/(dt_*dt_*dt_*dt_))*Qacc;
    ///setup Qjerk

    for (int i=0;i<2*length_-6;i++ ){
        Qjerk.s(20,i,i);
        Qjerk.s(-15,i,i+2);
        Qjerk.s(6,i,i+4);
        Qjerk.s(-1,i,i+6);
    }

    Qjerk.s(19,2*length_-6,2*length_-6);
    Qjerk.s(-12,2*length_-6,2*length_-4);
    Qjerk.s(-12,2*length_-4,2*length_-6);
    Qjerk.s(3,2*length_-6,2*length_-2);
    Qjerk.s(3,2*length_-2,2*length_-6);

    Qjerk.s(19,2*length_-5,2*length_-5);
    Qjerk.s(-12,2*length_-5,2*length_-3);
    Qjerk.s(-12,2*length_-3,2*length_-5);
    Qjerk.s(3,2*length_-5,2*length_-1);
    Qjerk.s(3,2*length_-1,2*length_-5);

    Qjerk.s(10,2*length_-4,2*length_-4);
    Qjerk.s(-3,2*length_-4,2*length_-2);
    Qjerk.s(-3,2*length_-2,2*length_-4);

    Qjerk.s(10,2*length_-3,2*length_-3);
    Qjerk.s(-3,2*length_-3,2*length_-1);
    Qjerk.s(-3,2*length_-1,2*length_-3);

    Qjerk.s(1,2*length_-2,2*length_-2);
    Qjerk.s(1,2*length_-1,2*length_-1);

    for(int i=0;i<2*length_-6;i++){
        for(int j=i;j<i+7;j++){
            Qjerk.s(Qjerk[i][j],j,i);
        }
    }
    //std::cout<<Qjerk<<std::endl;

    Qjerk=(1.0/(dt_*dt_*dt_*dt_*dt_*dt_))*Qjerk;

    ///setup Qd

    for (int i=4;i<2*length_;i = i+6){
        Qd.s(1,i,i);
        Qd.s(1,i+1,i+1);
    }

    ///setup Pacc
    Pjerk=0;
    Pacc.s(-2*vxinit/(dt_*dt_*dt_)-6*xinit/(dt_*dt_*dt_*dt_),0);
    Pacc.s(-2*vyinit/(dt_*dt_*dt_)-6*yinit/(dt_*dt_*dt_*dt_),1);
    Pacc.s(2*xinit/(dt_*dt_*dt_*dt_),2);
    Pacc.s(2*yinit/(dt_*dt_*dt_*dt_),3);

    ///setup Pjerk
    Pjerk=0;
    Pjerk.s(-2*axinit/(dt_*dt_*dt_*dt_)-8*vxinit/(dt_*dt_*dt_*dt_*dt_)-20*xinit/(dt_*dt_*dt_*dt_*dt_*dt_),0);
    Pjerk.s(-2*ayinit/(dt_*dt_*dt_*dt_)-8*vyinit/(dt_*dt_*dt_*dt_*dt_)-20*yinit/(dt_*dt_*dt_*dt_*dt_*dt_),1);
    Pjerk.s(2*vxinit/(dt_*dt_*dt_*dt_*dt_)+10*xinit/(dt_*dt_*dt_*dt_*dt_*dt_),2);
    Pjerk.s(2*vyinit/(dt_*dt_*dt_*dt_*dt_)+10*yinit/(dt_*dt_*dt_*dt_*dt_*dt_),3);
    Pjerk.s(-2*xinit/(dt_*dt_*dt_*dt_*dt_*dt_),4);
    Pjerk.s(-2*yinit/(dt_*dt_*dt_*dt_*dt_*dt_),5);

    ///setup Q P
    Q=wacc*Qacc+wdist*Qd+wjerk*Qjerk;
    P=wacc*Pacc+wjerk*Pjerk;
    Q=2.0*Q;
}
void smoother::setupAb(quadprogpp::Matrix<double> Aeq_, quadprogpp::Matrix<double> A_,quadprogpp::Vector<double> beq_,quadprogpp::Vector<double> b_){
    Aeq=Aeq_;
    A=A_;
    beq=beq_;
    b=b_;

}

void smoother::setting(int length,  double* init, double* weight, double* point,double dt){
    dt_=dt/3;
    double wdist = weight[2];
    std::vector<double> in;
    in.resize(6*length,0);
    for (int i = 0; i < length ; i++) {
        in[6*i+4]=point[2*i];
        in[6*i+5]=point[2*i+1];
    }
    length_=3*length;
    setupQP(init, weight);
    quadprogpp::Vector<double> f_dist;
    f_dist.resize(0,2*length_);
    f_dist=0;
    for(int i=0; i<2*length_;i++) f_dist.s(-2*in[i],i);
    P += wdist*f_dist;

}
void smoother::smoothing(quadprogpp::Vector<double>& x){
    x.resize(0,2*length_);
    std::cout<<"fval:"<<solve_quadprog(Q,P,Aeq,-beq,-A,b,x)<<std::endl;

}
