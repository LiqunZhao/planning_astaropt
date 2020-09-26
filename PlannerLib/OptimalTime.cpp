//
// Created by Chenran on 06/10/19.
//


#include "OptimalTime.hpp"
#include <eigen3/Eigen/Core>
#include "find_polynomial_roots_jenkins_traub.h"

void OptimalTime::ForStop(){

    double p8 = wA_ * pow(wT_,2);
    double p7 = 18 * wA_ * wT_;
    double p6 = 9 * (9 * wT_ - pow(acceleration_,2) * pow(wA_,2));
    double p5 = -144 * acceleration_ * wA_ * (acceleration_ + velocity_ * wA_);
    double p4 = -72 * (9*pow(acceleration_,2)+8*pow(velocity_,2)*pow(wA_,2) - 5*acceleration_*wA_*(dest_dis_*wA_-6*velocity_));
    double p3 = 144*(-7*acceleration_*(9*velocity_-5*dest_dis_*wA_)+4*velocity_*wA_*(-14*velocity_+5*dest_dis_*wA_));
    double p2 = 144*(135*acceleration_*dest_dis_-216*pow(velocity_,2)+260*dest_dis_*velocity_*wA_-25*pow(dest_dis_,2)*pow(wA_,2));
    double p1 = - 43200*dest_dis_*(-3*velocity_+dest_dis_*wA_);
    double p0 = -129600*pow(dest_dis_,2);

    Eigen::VectorXd polynomial;
    Eigen::VectorXd real_roots;
    Eigen::VectorXd complex_roots;
    polynomial << p8,p7,p6,p5,p4,p3,p2,p1,p0;
    rpoly_plus_plus::FindPolynomialRootsJenkinsTraub(polynomial, &real_roots, &complex_roots);
    double T = 10000000000000;
    for(int i = 0; i < real_roots.size();i++){
        if (real_roots[i] < T && real_roots[i]>0) T = real_roots[i];
    }
    /*
    double tmppre;
    while (true){
        double tmp =
            p8 * pow(T, 8) + p7 * pow(T, 7) + p6 * pow(T, 6) + p5 * pow(T, 5) + p4 * pow(T, 4) + p3 * pow(T, 3) +
            p2 * pow(T, 2) + p1 * T + p0;
        if (tmp>0) {
            tmppre = tmp;
            T = T-0.9999;
            break;
        }
        T = T+1;
    }
    while (true) {
        double tmp =
            p8 * pow(T, 8) + p7 * pow(T, 7) + p6 * pow(T, 6) + p5 * pow(T, 5) + p4 * pow(T, 4) + p3 * pow(T, 3) +
            p2 * pow(T, 2) + p1 * T + p0;
        if (tmp>0) {
            if (std::abs(tmp) > std::abs(tmppre)) T=T-0.0001;
            break;
        }
        T += 0.0001;
        tmppre = tmp;
    }
    */
    double t = T;
    double af = 3*(8*t*velocity_ + acceleration_*pow(t,2) - 20*dest_dis_)/(pow(t, 2)*(9+t*wA_));
    c1 = velocity_;
    c2 = acceleration_;
    c3 = 60*dest_dis_/pow(t, 3) - 36*velocity_/pow(t, 2) + 3*(af-3*acceleration_)/t;
    c4 = -360*dest_dis_/pow(t, 4) + 192*velocity_/pow(t, 3) + 6*(6*acceleration_-4*af)/pow(t, 2);
    c5 = 720*dest_dis_/pow(t, 5) - 360*velocity_/pow(t, 4) + 60*(af-acceleration_)/pow(t, 3);
    optimal_T =t;
}

void OptimalTime::FreeFlow(){

    double p4 = wT_;
    double p2 = -4*acceleration_*acceleration_;
    double p3 = 0;
    double p1 = 24*acceleration_*(desired_velocity_-velocity_);
    double p0 = -36*(desired_velocity_-velocity_)*(desired_velocity_-velocity_);

    Eigen::VectorXd polynomial;
    Eigen::VectorXd real_roots;
    Eigen::VectorXd complex_roots;
    polynomial << p4,p3,p2,p1,p0;
    rpoly_plus_plus::FindPolynomialRootsJenkinsTraub(polynomial, &real_roots, &complex_roots);
    double T = 10000000000000;
    for(int i = 0; i < real_roots.size();i++){
        if (real_roots[i] < T && real_roots[i]>0) T = real_roots[i];
    }

    /*
    double T = 0.0f;
    double tmppre;
    while (true){
        double tmp =
                p4 * pow(T, 4) + p2 * pow(T, 2) + p1 * T + p0;
        if (tmp>0) {
            tmppre = tmp;
            T = T-0.9999;
            break;
        }
        T = T+1;
    }
    while (true) {
        double tmp =
                p4 * pow(T, 4) + p2 * pow(T, 2) + p1 * T + p0;
        if (tmp>0) {
            if (std::abs(tmp) > std::abs(tmppre)) T=T-0.0001;
            break;
        }
        T += 0.0001;
        tmppre = tmp;
    }

    */


    double t = T;
    c1 = velocity_;
    c2 = acceleration_;
    c3 = 6*(desired_velocity_-velocity_)/pow(t,2)-4*acceleration_/t;
    c4 = 6*acceleration_/pow(t,2)-12*(desired_velocity_-velocity_)/pow(t,3);
    c5 = 0;
}

std::vector<double> OptimalTime::getstate(double t){
    std::vector<double> states;
    double next_lng_pos = lng_pos_+c1*t+c2*pow(t,2)/2+c3*pow(t,3)/6+c4*pow(t,4)/24+c5*pow(t,5)/120;
    double next_v = c1+c2*t+c3*pow(t,2)/2+c4*pow(t,3)/6+c5*pow(t,4)/24;
    double next_a = c2+c3*t+c4*pow(t,2)/2+c5*pow(t,3)/6;
    states.push_back(next_lng_pos);
    states.push_back(next_v);
    states.push_back(next_a);
    return states;
}