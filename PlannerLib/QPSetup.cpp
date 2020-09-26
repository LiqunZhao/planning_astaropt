//
// Created by 雷宇恒 on 2018/8/11.
//
#include <iostream>
#include "Array.hh"
#include "QPSetup.h"
#include <cmath>

void QPSetup(double* init, double* weight, double dt, unsigned int Tp, unsigned int K, quadprogpp::Matrix<double>& Q, quadprogpp::Matrix<double>& Qv,
             quadprogpp::Matrix<double>& Av, quadprogpp::Vector<double>& p, quadprogpp::Vector<double>& pv, quadprogpp::Vector<double>& bv) {
    double xinit = init[0];
    double yinit = init[1];
    double vxinit = init[2];
    double vyinit = init[3];
    double axinit = init[4];
    double ayinit = init[5];
    double wacc = weight[0];
    double wjerk = weight[1];
    double wdist = weight[2];
    double wvacc = weight[3];
    double wvjerk = weight[4];
    double wvdist = weight[5];
    unsigned int KT = K * Tp;

    //acceleration penalty
    quadprogpp::Matrix<double> Qacc(KT, KT);
    quadprogpp::Vector<double> pacc(KT);
    Qacc = 0;
    pacc = 0;
    Qacc.s(1, 0, 0);
    Qacc.s(1, 1, 1);
    Qacc.s(1, KT - 1, KT - 1);
    Qacc.s(1, KT - 2, KT - 2);
    Qacc.s(5, K, K);
    Qacc.s(5, K + 1, K + 1);
    Qacc.s(5, KT - 3, KT - 3);
    Qacc.s(5, KT - 4, KT - 4);
    for (int i = 2 * K; i <= KT - 3 * K; i += K) {
        Qacc.s(6, i, i);
        Qacc.s(6, i + 1, i + 1);
    }
    Qacc.s(-2, 0, K);
    Qacc.s(-2, 1, K + 1);
    Qacc.s(-2, K, 0);
    Qacc.s(-2, K + 1, 1);
    Qacc.s(-2, KT - 1, KT - K - 1);
    Qacc.s(-2, KT - 2, KT - K - 2);
    Qacc.s(-2, KT - K - 1, KT - 1);
    Qacc.s(-2, KT - K - 2, KT - 2);
    for (int i = K; i <= KT - 3 * K; i += K) {
        Qacc.s(-4, i, i + K);
        Qacc.s(-4, i + K, i);
        Qacc.s(-4, i + 1, i + K + 1);
        Qacc.s(-4, i + K + 1, i + 1);
    }
    for (int i = 0; i <= KT - 3 * K; i += K) {
        Qacc.s(1, i, i + 2 * K);
        Qacc.s(1, i + 2 * K, i);
        Qacc.s(1, i + 1, i + 2 * K + 1);
        Qacc.s(1, i + 2 * K + 1, i + 1);
    }
    //for a1
    Qacc.add(1, 0, 0);
    Qacc.add(1, 1, 1);
    pacc.s(-2 * (xinit + vxinit * dt), 0);
    pacc.s(-2 * (yinit + vyinit * dt), 1);
    //for a2
    Qacc.add(4, 0, 0);
    Qacc.add(4, 1, 1);
    Qacc.add(1, K, K);
    Qacc.add(1, K + 1, K + 1);
    Qacc.add(-2, 0, K);
    Qacc.add(-2, 1, K + 1);
    Qacc.add(-2, K, 0);
    Qacc.add(-2, K + 1, 1);
    pacc.add(-4 * xinit, 0);
    pacc.add(-4 * yinit, 1);
    pacc.add(2*xinit,2);
    pacc.add(2*yinit,3);

    Qacc /= pow(dt, 4);
    pacc /= pow(dt, 4);

    // jerk penalty
    quadprogpp::Matrix<double> Qjerk(KT, KT);
    quadprogpp::Vector<double> pjerk(KT);
    Qjerk = 0;
    pjerk = 0;
    Qjerk.s(1, 0, 0);
    Qjerk.s(1, 1, 1);
    Qjerk.s(1, KT - 1, KT - 1);
    Qjerk.s(1, KT - 2, KT - 2);
    Qjerk.s(10, K, K);
    Qjerk.s(10, K + 1, K + 1);
    Qjerk.s(10, KT - 2 * K + 1, KT - 2 * K + 1);
    Qjerk.s(10, KT - 2 * K, KT - 2 * K);
    Qjerk.s(19, 2 * K, 2 * K);
    Qjerk.s(19, 2 * K + 1, 2 * K + 1);
    Qjerk.s(19, KT - 3 * K + 1, KT - 3 * K + 1);
    Qjerk.s(19, KT - 3 * K, KT - 3 * K);
    for (int i = 3 * K; i <= KT - 4 * K; i += K) {
        Qjerk.s(20, i, i);
        Qjerk.s(20, i + 1, i + 1);
    }
    // 1-step cross
    Qjerk.s(-3, 0, K);
    Qjerk.s(-3, 1, K + 1);
    Qjerk.s(-3, K, 0);
    Qjerk.s(-3, K + 1, 1);
    Qjerk.s(-3, KT - 2 * K, KT - K);
    Qjerk.s(-3, KT - 2 * K + 1, KT - K + 1);
    Qjerk.s(-3, KT - K, KT - 2 * K);
    Qjerk.s(-3, KT - K + 1, KT - 2 * K + 1);
    Qjerk.s(-12, K, 2 * K);
    Qjerk.s(-12, K + 1, 2 * K + 1);
    Qjerk.s(-12, 2 * K, K);
    Qjerk.s(-12, 2 * K + 1, K + 1);
    Qjerk.s(-12, KT - 3 * K, KT - 2 * K);
    Qjerk.s(-12, KT - 3 * K + 1, KT - 2 * K + 1);
    Qjerk.s(-12, KT - 2 * K, KT - 3 * K);
    Qjerk.s(-12, KT - 2 * K + 1, KT - 3 * K + 1);
    for (int i = 2 * K; i <= KT - 4 * K; i += K) {
        Qjerk.s(-15, i, i + K);
        Qjerk.s(-15, i + K, i);
        Qjerk.s(-15, i + 1, i + K + 1);
        Qjerk.s(-15, i + K + 1, i + 1);
    }
    // 2-step cross
    Qjerk.s(3, 0, 2 * K);
    Qjerk.s(3, 1, 2 * K + 1);
    Qjerk.s(3, 2 * K, 0);
    Qjerk.s(3, 2 * K + 1, 1);
    Qjerk.s(3, KT - 3 * K, KT - K);
    Qjerk.s(3, KT - 3 * K + 1, KT - K + 1);
    Qjerk.s(3, KT - K, KT - 3 * K);
    Qjerk.s(3, KT - K + 1, KT - 3 * K + 1);
    for (int i = K; i <= KT - 4 * K; i += K) {
        Qjerk.s(6, i, i + 2 * K);
        Qjerk.s(6, i + 2 * K, i);
        Qjerk.s(6, i + 1, i + 2 * K + 1);
        Qjerk.s(6, i + 2 * K + 1, i + 1);
    }
    // 3-step cross
    for (int i = 0; i <= KT - 4 * K; i += K) {
        Qjerk.s(-1, i, i + 3 * K);
        Qjerk.s(-1, i + 3 * K, i);
        Qjerk.s(-1, i + 1, i + 3 * K + 1);
        Qjerk.s(-1, i + 3 * K + 1, i + 1);
    }
    // j1
    Qjerk.add(1, 0, 0);
    Qjerk.add(1, 1, 1);
    pjerk.add(-2 * (xinit + vxinit * dt + axinit * pow(dt, 2)), 0);
    pjerk.add(-2 * (yinit + vyinit * dt + ayinit * pow(dt, 2)), 1);
    // j2
    Qjerk.add(9, 0, 0);
    Qjerk.add(9, 1, 1);
    Qjerk.add(1, K, K);
    Qjerk.add(1, K + 1, K + 1);
    Qjerk.add(-3, 0, K);
    Qjerk.add(-3, 1, K + 1);
    Qjerk.add(-3, K, 0);
    Qjerk.add(-3, K + 1, 1);
    pjerk.add(-6 * (2 * xinit + vxinit * dt), 0);
    pjerk.add(-6 * (2 * yinit + vyinit * dt), 1);
    pjerk.add(2 * (2 * xinit + vxinit * dt), 2);
    pjerk.add(2 * (2 * yinit + vyinit * dt), 3);
    // j3
    Qjerk.add(9, 0, 0);
    Qjerk.add(9, 1, 1);
    Qjerk.add(9, K, K);
    Qjerk.add(9, K + 1, K + 1);
    Qjerk.add(1, 2 * K, 2 * K);
    Qjerk.add(1, 2 * K + 1, 2 * K + 1);
    Qjerk.add(-9, 0, K);
    Qjerk.add(-9, 1, K + 1);
    Qjerk.add(-9, K, 0);
    Qjerk.add(-9, K + 1, 1);
    Qjerk.add(-3, K, 2 * K);
    Qjerk.add(-3, K + 1, 2 * K + 1);
    Qjerk.add(-3, 2 * K, K);
    Qjerk.add(-3, 2 * K + 1, K + 1);
    Qjerk.add(3, 0, 2 * K);
    Qjerk.add(3, 1, 2 * K + 1);
    Qjerk.add(3, 2 * K, 0);
    Qjerk.add(3, 2 * K + 1, 1);
    pjerk.add(-6 * xinit, 0);
    pjerk.add(-6 * yinit, 1);
    pjerk.add(6 * xinit, 2);
    pjerk.add(6 * yinit, 3);
    pjerk.add(-2 * xinit, 4);
    pjerk.add(-2 * yinit, 5);

    Qjerk /= pow(dt, 6);
    pjerk /= pow(dt, 6);
    // distance
    quadprogpp::Matrix<double> Qdist(KT, KT);
    Qdist = 0;
    for (int i = 3; i <= Tp;i=i+3) {
        Qdist.s(1, K * i - K, K * i - K);
        Qdist.s(1, K * i - 1, K * i - 1);
    }



    quadprogpp::Vector<double> pdist(KT);
    pdist = 0;

    Q = wacc * Qacc + wjerk * Qjerk + wdist * Qdist;
    Q *= 2;
    p = wacc * pacc + wjerk * pjerk;


    //
    xinit = 0;
    yinit = 0;
    vxinit = sqrt(pow(init[2], 2) + pow(init[3], 2));
    vyinit = 0;
    axinit = sqrt(pow(init[4], 2) + pow(init[5], 2));
    ayinit = 0;
    //acceleration penalty
    Qacc = 0;
    pacc = 0;
    Qacc.s(1, 0, 0);
    Qacc.s(1, 1, 1);
    Qacc.s(1, KT - 1, KT - 1);
    Qacc.s(1, KT - 2, KT - 2);
    Qacc.s(5, K, K);
    Qacc.s(5, K + 1, K + 1);
    Qacc.s(5, KT - 3, KT - 3);
    Qacc.s(5, KT - 4, KT - 4);
    for (int i = 2 * K; i <= KT - 3 * K; i += K) {
        Qacc.s(6, i, i);
        Qacc.s(6, i + 1, i + 1);
    }
    Qacc.s(-2, 0, K);
    Qacc.s(-2, 1, K + 1);
    Qacc.s(-2, K, 0);
    Qacc.s(-2, K + 1, 1);
    Qacc.s(-2, KT - 1, KT - K - 1);
    Qacc.s(-2, KT - 2, KT - K - 2);
    Qacc.s(-2, KT - K - 1, KT - 1);
    Qacc.s(-2, KT - K - 2, KT - 2);
    for (int i = K; i <= KT - 3 * K; i += K) {
        Qacc.s(-4, i, i + K);
        Qacc.s(-4, i + K, i);
        Qacc.s(-4, i + 1, i + K + 1);
        Qacc.s(-4, i + K + 1, i + 1);
    }
    for (int i = 0; i <= KT - 3 * K; i += K) {
        Qacc.s(1, i, i + 2 * K);
        Qacc.s(1, i + 2 * K, i);
        Qacc.s(1, i + 1, i + 2 * K + 1);
        Qacc.s(1, i + 2 * K + 1, i + 1);
    }
    //for a1
    Qacc.add(1, 0, 0);
    Qacc.add(1, 1, 1);
    pacc.s(-2 * (xinit + vxinit * dt), 0);
    pacc.s(-2 * (yinit + vyinit * dt), 1);
    //for a2
    Qacc.add(4, 0, 0);
    Qacc.add(4, 1, 1);
    Qacc.add(1, K, K);
    Qacc.add(1, K + 1, K + 1);
    Qacc.add(-2, 0, K);
    Qacc.add(-2, 1, K + 1);
    Qacc.add(-2, K, 0);
    Qacc.add(-2, K + 1, 1);
    pacc.add(-4 * xinit, 0);
    pacc.add(-4 * yinit, 1);

    Qacc /= pow(dt, 4);
    pacc /= pow(dt, 4);

    // jerk penalty
    Qjerk = 0;
    pjerk = 0;
    Qjerk.s(1, 0, 0);
    Qjerk.s(1, 1, 1);
    Qjerk.s(1, KT - 1, KT - 1);
    Qjerk.s(1, KT - 2, KT - 2);
    Qjerk.s(10, K, K);
    Qjerk.s(10, K + 1, K + 1);
    Qjerk.s(10, KT - 2 * K + 1, KT - 2 * K + 1);
    Qjerk.s(10, KT - 2 * K, KT - 2 * K);
    Qjerk.s(19, 2 * K, 2 * K);
    Qjerk.s(19, 2 * K + 1, 2 * K + 1);
    Qjerk.s(19, KT - 3 * K + 1, KT - 3 * K + 1);
    Qjerk.s(19, KT - 3 * K, KT - 3 * K);
    for (int i = 3 * K; i <= KT - 4 * K; i += K) {
        Qjerk.s(20, i, i);
        Qjerk.s(20, i + 1, i + 1);
    }
    // 1-step cross
    Qjerk.s(-3, 0, K);
    Qjerk.s(-3, 1, K + 1);
    Qjerk.s(-3, K, 0);
    Qjerk.s(-3, K + 1, 1);
    Qjerk.s(-3, KT - 2 * K, KT - K);
    Qjerk.s(-3, KT - 2 * K + 1, KT - K + 1);
    Qjerk.s(-3, KT - K, KT - 2 * K);
    Qjerk.s(-3, KT - K + 1, KT - 2 * K + 1);
    Qjerk.s(-12, K, 2 * K);
    Qjerk.s(-12, K + 1, 2 * K + 1);
    Qjerk.s(-12, 2 * K, K);
    Qjerk.s(-12, 2 * K + 1, K + 1);
    Qjerk.s(-12, KT - 3 * K, KT - 2 * K);
    Qjerk.s(-12, KT - 3 * K + 1, KT - 2 * K + 1);
    Qjerk.s(-12, KT - 2 * K, KT - 3 * K);
    Qjerk.s(-12, KT - 2 * K + 1, KT - 3 * K + 1);
    for (int i = 2 * K; i <= KT - 4 * K; i += K) {
        Qjerk.s(-15, i, i + K);
        Qjerk.s(-15, i + K, i);
        Qjerk.s(-15, i + 1, i + K + 1);
        Qjerk.s(-15, i + K + 1, i + 1);
    }
    // 2-step cross
    Qjerk.s(3, 0, 2 * K);
    Qjerk.s(3, 1, 2 * K + 1);
    Qjerk.s(3, 2 * K, 0);
    Qjerk.s(3, 2 * K + 1, 1);
    Qjerk.s(3, KT - 3 * K, KT - K);
    Qjerk.s(3, KT - 3 * K + 1, KT - K + 1);
    Qjerk.s(3, KT - K, KT - 3 * K);
    Qjerk.s(3, KT - K + 1, KT - 3 * K + 1);
    for (int i = K; i <= KT - 4 * K; i += K) {
        Qjerk.s(6, i, i + 2 * K);
        Qjerk.s(6, i + 2 * K, i);
        Qjerk.s(6, i + 1, i + 2 * K + 1);
        Qjerk.s(6, i + 2 * K + 1, i + 1);
    }
    // 3-step cross
    for (int i = 0; i <= KT - 4 * K; i += K) {
        Qjerk.s(-1, i, i + 3 * K);
        Qjerk.s(-1, i + 3 * K, i);
        Qjerk.s(-1, i + 1, i + 3 * K + 1);
        Qjerk.s(-1, i + 3 * K + 1, i + 1);
    }
    // j1
    Qjerk.add(1, 0, 0);
    Qjerk.add(1, 1, 1);
    pjerk.add(-2 * (xinit + vxinit * dt + axinit * pow(dt, 2)), 0);
    pjerk.add(-2 * (yinit + vyinit * dt + ayinit * pow(dt, 2)), 1);
    // j2
    Qjerk.add(9, 0, 0);
    Qjerk.add(9, 1, 1);
    Qjerk.add(1, K, K);
    Qjerk.add(1, K + 1, K + 1);
    Qjerk.add(-3, 0, K);
    Qjerk.add(-3, 1, K + 1);
    Qjerk.add(-3, K, 0);
    Qjerk.add(-3, K + 1, 1);
    pjerk.add(-6 * (2 * xinit + vxinit * dt), 0);
    pjerk.add(-6 * (2 * yinit + vyinit * dt), 1);
    pjerk.add(2 * (2 * xinit + vxinit * dt), 2);
    pjerk.add(2 * (2 * yinit + vyinit * dt), 3);
    // j3
    Qjerk.add(9, 0, 0);
    Qjerk.add(9, 1, 1);
    Qjerk.add(9, K, K);
    Qjerk.add(9, K + 1, K + 1);
    Qjerk.add(1, 2 * K, 2 * K);
    Qjerk.add(1, 2 * K + 1, 2 * K + 1);
    Qjerk.add(-9, 0, K);
    Qjerk.add(-9, 1, K + 1);
    Qjerk.add(-9, K, 0);
    Qjerk.add(-9, K + 1, 1);
    Qjerk.add(-3, K, 2 * K);
    Qjerk.add(-3, K + 1, 2 * K + 1);
    Qjerk.add(-3, 2 * K, K);
    Qjerk.add(-3, 2 * K + 1, K + 1);
    Qjerk.add(3, 0, 2 * K);
    Qjerk.add(3, 1, 2 * K + 1);
    Qjerk.add(3, 2 * K, 0);
    Qjerk.add(3, 2 * K + 1, 1);
    pjerk.add(-6 * xinit, 0);
    pjerk.add(-6 * yinit, 1);
    pjerk.add(6 * xinit, 2);
    pjerk.add(6 * yinit, 3);
    pjerk.add(-2 * xinit, 4);
    pjerk.add(-2 * yinit, 5);

    Qjerk /= pow(dt, 6);
    pjerk /= pow(dt, 6);
    // distance
    Qdist = 0;
    for (int i = 1; i <= Tp; i++) {
        Qdist.s(1, K * i - K, K * i - K);
        Qdist.s(1, K * i - 1, K * i - 1);
    }
    pdist = 0;

    // speed profile optimization
    quadprogpp::Matrix<double> Qvacc(Tp, Tp);
    quadprogpp::Vector<double> pvacc(Tp);
    quadprogpp::Matrix<double> Qvjerk(Tp, Tp);
    quadprogpp::Vector<double> pvjerk(Tp);
    quadprogpp::Matrix<double> Qvdist(Tp, Tp);
    Qvacc = 0;
    pvacc = 0;
    Qvjerk = 0;
    pvjerk = 0;
    Qvdist = 0;
    for (int i = 1; i <= Tp; i++) {
        pvacc.s(pacc[i * K - 2], i - 1);
        pvjerk.s(pjerk[i * K - 2], i - 1);
    }
    for (int i = 1; i <= Tp; i++) {
        for (int j = 1; j <= Tp; j++) {
            Qvacc.s(Qacc[i * K - 2][j * K - 1], i - 1, j - 1);
            Qvjerk.s(Qjerk[i * K - 2][j * K - 1], i - 1, j - 1);
        }
    }
    for (int i = 3; i <= Tp; i += 3) {
        Qvdist.s(1, i - 1, i - 1);
    }


    Qv = wvacc * Qvacc + wvjerk * Qvjerk + wvdist * Qvdist;
    Qv *= 2;
    pv = wvacc * pvacc + wvjerk * pvjerk;

    // constraints
    Av.s(-1, 0, 0);
    for (int i = 1; i <= Tp - 1; i++) {
        Av.s(-1, i, i);
        Av.s(1, i, i - 1);
    }
};
