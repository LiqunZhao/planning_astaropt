//
//  vehicle.cpp
//  MOBIL
//
//  Created by Ao LI on 26/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#include "vehicle.hpp"

void vehicle::calControlSolution() {
    this->calCoefficients();
}

void vehicle::setSuccessor(vehicle* succ){
    this->successor_ = succ;
}

void vehicle::setPrecessor(vehicle* prec){
    this->precessor_ = prec;
}

void vehicle::setDestinationDistance(float dis){
    this->dest_dis_ = dis;
}

float vehicle::findOptimalTime(){
    float p8 = wA_ * pow(wT_,2);
    float p7 = 18 * wA_ * wT_;
    float p6 = 9 * (9 * wT_ - pow(acceleration_,2) * pow(wA_,2));
    float p5 = -144 * acceleration_ * wA_ * (acceleration_ + velocity_ * wA_);
    float p4 = -72 * (9*pow(acceleration_,2)+8*pow(velocity_,2)*pow(wA_,2) - 5*acceleration_*wA_*(dest_dis_*wA_-6*velocity_));
    float p3 = 144*(-7*acceleration_*(9*velocity_-5*dest_dis_*wA_)+4*velocity_*wA_*(-14*velocity_+5*dest_dis_*wA_));
    float p2 = 144*(135*acceleration_*dest_dis_-216*pow(velocity_,2)+260*dest_dis_*velocity_*wA_-25*pow(dest_dis_,2)*pow(wA_,2));
    float p1 = - 43200*dest_dis_*(-3*velocity_+dest_dis_*wA_);
    float p0 = -129600*pow(dest_dis_,2);
    
    float T = 0.0f;
    float min = std::abs(p0);
    float res = T;
    while (T < 50) {
        float tmp = p8*pow(T, 8) + p7*pow(T, 7) + p6*pow(T, 6) + p5*pow(T, 5) + p4*pow(T, 4) + p3*pow(T, 3) + p2*pow(T, 2) + p1*T + p0;
        if (std::abs(tmp) < min) {
            min = std::abs(tmp);
            res = T;
        }
        T+=0.0001;
    }
    return res;
}

void vehicle::calCoefficients(){
    float T = this->findOptimalTime();
    float af = 3*(8*T*velocity_ + acceleration_*pow(T,2) - 20*dest_dis_)/(pow(T, 2)*(9+T*wA_));
    c1 = velocity_;
    c2 = acceleration_;
    c3 = 60*dest_dis_/pow(T, 3) - 36*velocity_/pow(T, 2) + 3*(af-3*acceleration_)/T;
    c4 = -360*dest_dis_/pow(T, 4) + 192*velocity_/pow(T, 3) + 6*(6*acceleration_-4*af)/pow(T, 2);
    c5 = 720*dest_dis_/pow(T, 5) - 360*velocity_/pow(T, 4) + 60*(af-acceleration_)/pow(T, 3);
}


/**********************
 
  -----          -----         -----
 |  n  |        |     |       |  s  |
  -----          -----         -----
 ===========================================   Lane line
                  ^
                  |
  -----        -----          -----
 |  o  |      |  c  |        |  q  |
  -----        -----          -----
 
 **********************/


void vehicle::changeLane(vehicle* o, vehicle* n, vehicle* q, vehicle* s) {
    if(is_changing_!=0) return;
    if (current_lane_ == left) {
        current_lane_ = right;
        is_changing_ = -1;
        sig_value_ = -4;
    }
    else {
        current_lane_ = left;
        is_changing_ = 1;
        sig_value_ = 4;
    }
    this->setPrecessor(s);
    this->setSuccessor(n);
    o->setPrecessor(q);
    q->setSuccessor(o);
    n->setPrecessor(this);
    s->setSuccessor(this);
    
}

void vehicle::updateStatus() {
    float T = TIME_GAP * (++time_count_);
    this->desired_velocity_ = c1 + c2 * T
                              + (1.0/2) * c3 * pow(T, 2)
                              + (1.0/6) * c4 * pow(T, 3)
                              + (1.0/24) * c5 * pow(T, 4);

    acceleration_ = predictAcceleration(this->precessor_);
    velocity_ += acceleration_ * TIME_GAP;
    velocity_ = fmax(velocity_, 0);
    lng_pos_ += velocity_ * TIME_GAP;
    
    if(is_changing_ == -1) {
        lat_pos_ =  1/(1+exp(-sig_value_));
        sig_value_ += 1;
        if (sig_value_ > 4) {
            lat_pos_ = 1;
            is_changing_ = 0;
        }
    }
    else if (is_changing_ == 1){
        lat_pos_ =  1/(1+exp(-sig_value_));
        sig_value_ -= 1;
        if (sig_value_ < -4) {
            lat_pos_ = 0;
            is_changing_ = 0;
        }
    }
}

/*
 This function calculate the a_tilde
 */

float vehicle::predictAcceleration (vehicle* prec) {
    if (!prec) return (MAX_ACC * (1 - pow((velocity_/desired_velocity_),4)));
    double s_star = MIN_DISTANCE + this->velocity_ * TIME_GAP + this->velocity_ / (2 * sqrt(MAX_ACC * (-DESIRED_DECC)));
    double s_alpha = prec->lng_pos_ - this->lng_pos_; // Current distance to the precessor
    float res = MAX_ACC * (1 - pow((velocity_/desired_velocity_), 4) - pow(s_star/s_alpha, 2));

    if (res > MAX_ACC) res = MAX_ACC;
    if (res < DESIRED_DECC) res = DESIRED_DECC;
    return res;
}

float vehicle::convertByEuropeRules(float a_tilde, vehicle* prec) {
    // According to the passing rule
    return ((velocity_ > prec->velocity_) && (prec->velocity_ > CRITICAL_VELOCITY))?
    std::min(a_tilde, acceleration_) : a_tilde;
}

/*
 For asymmetric passing rules
 L -> R: a_eur_tilde - ac + p * (ao_tilde - ao) > delta_a_th - delta_a_bias
 R -> L: ac_tilde - ac_eur + p * (an_tilde - an) > delta_a_th + delta_a_bias
 
 For systemetric passing rules:
 if (ac_tilde - ac) + p * ((an_tilde - an) + (ao_tilde - ao))
      driver              new_follower       old_follower
 */

bool vehicle::getLaneChangingDecision(vehicle* n, vehicle* s) {
    if (this->acceleration_ < 0) return false; // We do not change lane during deccelerating
    
    vehicle* succ = this->successor_;
    vehicle* prec = this->precessor_;
    
    float ac_tilde = predictAcceleration(s);
    float ac = predictAcceleration(this->precessor_);
    float ao_tilde = succ->predictAcceleration(prec);
    float ao = this->successor_->acceleration_;
    float an_tilde = n->predictAcceleration(this);
    float an = n->acceleration_;
    float ac_eur_tilde = convertByEuropeRules(ac_tilde, n);
    float ac_eur = convertByEuropeRules(ac, n);
    
    if (n->lng_pos_ + MIN_DISTANCE > this->lng_pos_ || s->lng_pos_ - MIN_DISTANCE < this->lng_pos_)
        return false; // To ensure the min distance bewteen two cars
    
    if (current_lane_ == left) {    // This vehicle is currently on the left lane
        return ((ac_eur_tilde - ac + POLITENESS * (ao_tilde - ao)) > (CHANGEING_THRESHOLD - BIAS_FOR_RIHGT_LANE))?
        true : false;
    } else {
        return ((ac_tilde - ac_eur + POLITENESS * (an_tilde - an)) > (CHANGEING_THRESHOLD + BIAS_FOR_RIHGT_LANE))?
        true : false;
    }
}
