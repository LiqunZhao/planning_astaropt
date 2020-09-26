//
//  vehicle.cpp
//  MOBIL
//
//  Created by Ao LI on 26/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#include "vehicle.hpp"

void lerp(BasicPoint2d& dest, const BasicPoint2d& a, const BasicPoint2d& b, const float t)
{
    dest = a + (b-a)*t;
}

// evaluate a BasicPoint2d on a bezier-curve. t goes from 0 to 1.0
void bezier(BasicPoint2d &dest, const BasicPoint2d& a, const BasicPoint2d& b, const BasicPoint2d& c, const BasicPoint2d& d, const float t)
{
    BasicPoint2d ab,bc,cd,abbc,bccd;
    lerp(ab, a,b,t);           // BasicPoint2d between a and b (green)
    lerp(bc, b,c,t);           // BasicPoint2d between b and c (green)
    lerp(cd, c,d,t);           // BasicPoint2d between c and d (green)
    lerp(abbc, ab,bc,t);       // BasicPoint2d between ab and bc (blue)
    lerp(bccd, bc,cd,t);       // BasicPoint2d between bc and cd (blue)
    lerp(dest, abbc,bccd,t);   // BasicPoint2d on the bezier-curve (black)
}


void vehicle::setSuccessor(vehicle* succ){
    this->successor_ = succ;
}

void vehicle::setPrecessor(vehicle* prec){
    this->precessor_ = prec;
}

void vehicle::setTraveledDistanceInCurrLanelet(float dis) {
    this->traveled_dis_ = dis;
}

void vehicle::setLaneletId(int laneletId) {
    this->currentLaneletId_ = laneletId;
}

void vehicle::setRoutingLanelet(ConstLanelet& startLanelet, ConstLanelet& destinationLanelet){
    startLaneletId_ = startLanelet.id();
    destinationLaneletId_ = destinationLanelet.id();
    startLanelet_ = startLanelet;
    destinationLanelet_ = destinationLanelet;
}

void vehicle::setShortestPath(const routing::LaneletPath& shortestPath){
    shortestPath_ = std::move(shortestPath);
}

int vehicle::getNextLaneletId(){
    for (ConstLanelets::iterator iter = shortestPath_.begin(); iter != shortestPath_.end(); iter++) {
        if (currentLaneletId_ == iter->id() && iter != (shortestPath_.end()-1)) {
            return (++iter)->id();
        }
    }
    return 0;
}

// /**********************
 
//   -----          -----         -----
//  |  n  |        |     |       |  s  |
//   -----          -----         -----
//  ===========================================   Lane line
//                   ^
//                   |
//   -----        -----          -----
//  |  o  |      |  c  |        |  q  |
//   -----        -----          -----
 
//  **********************/

void vehicle::changeLane(vehicle* o, vehicle* n, vehicle* q, vehicle* s, int newLaneletId, float newDis) {
    this->setPrecessor(s);
    this->setSuccessor(n);
    this->setLaneletId(newLaneletId);
    this->setTraveledDistanceInCurrLanelet(newDis);
    if(o) o->setPrecessor(q);
    if(q) q->setSuccessor(o);
    if(n) n->setPrecessor(this);
    if(s) s->setSuccessor(this);
    is_changing_ = true;
}

void vehicle::updateStatus() {
    acceleration_ = IDMAcceleration(this->precessor_);
    velocity_ += acceleration_ * TIME_GAP;
    velocity_ = fmax(velocity_, 0);
    if (is_changing_) {
        BasicPoint2d currPosition;
        bezier(currPosition, a, b, c, d, t);
        setPosition(currPosition);
        // BasicPoint2d pDiff = this->position() - d;
        // double dx = sqrt(pDiff.transpose() * pDiff);
        std::cout << id_ << "," << position_(0,0) << "," << position_(1,0) << "," << heading_ << std::endl;
        if (t >= 0.95) {
            is_changing_ = false;
            traveled_dis_ += 3;
            t = 0.05f;
        }
        t += 0.05;
        return;
    }

    traveled_dis_ += velocity_ * TIME_GAP;
    std::cout << id_ << "," << position_(0,0) << "," << position_(1,0) << "," << heading_ << std::endl;
}

/*
 This function calculate the a_tilde
 */

float vehicle::IDMAcceleration (vehicle* prec) {
    if (!prec) return (MAX_ACC * (1 - pow((velocity_/desired_velocity_),4)));
    double s_star = MIN_DISTANCE + this->velocity_ * TIME_GAP + this->velocity_ / (2 * sqrt(MAX_ACC * (-MAX_DECC)));
    BasicPoint2d pDiff = this->position() - prec->position();
    double s_alpha = sqrt(pDiff.transpose() * pDiff);

    float res = MAX_ACC * (1 - pow((velocity_/desired_velocity_), 4) - pow(s_star/s_alpha, 2));

    if (res > MAX_ACC) res = MAX_ACC;    // TODO
    if (res < MAX_DECC) res = MAX_DECC;

    return res;
}

float vehicle::OVMAcceleration (vehicle* prec) {
    if (!prec) return (MAX_ACC * (1 - pow((velocity_/desired_velocity_),4)));
    BasicPoint2d pDiff = this->position() - prec->position();
    double dx = sqrt(pDiff.transpose() * pDiff);
    float res = MAX_ACC * (V(dx) - velocity_);

    if (res > MAX_ACC) res = MAX_ACC;
    if (res < MAX_DECC) res = MAX_DECC;

    return res;
}

float vehicle::GFMAcceleration (vehicle* prec) {
    if (!prec) return (MAX_ACC * (1 - pow((velocity_/desired_velocity_),4)));
    BasicPoint2d pDiff = this->position() - prec->position();
    double dx = sqrt(pDiff.transpose() * pDiff);
    double delta_v = velocity_ - prec->velocity();
    int headviside = delta_v < 0? 0 : 1;
    float res = (V(dx) - desired_velocity_)/2.45 - (delta_v)*headviside*exp(-dx/98.78)/0.77;

    if (res > MAX_ACC) res = MAX_ACC;
    if (res < MAX_DECC) res = MAX_DECC;

    return res;
}

float vehicle::convertByEuropeRules(float a_tilde, vehicle* prec) {
    // According to the passing rule
    if (!prec) return a_tilde;

    return ((velocity_ > prec->velocity()) && (prec->velocity() > CRITICAL_VELOCITY))?
    std::min(a_tilde, acceleration_) : a_tilde;
}

// /*
//  For asymmetric passing rules
//  L -> R: a_eur_tilde - ac + p * (ao_tilde - ao) > delta_a_th - delta_a_bias
//  R -> L: ac_tilde - ac_eur + p * (an_tilde - an) > delta_a_th + delta_a_bias
 
//  For systemetric passing rules:
//  if (ac_tilde - ac) + p * ((an_tilde - an) + (ao_tilde - ao))
//       driver              new_follower       old_follower
//  */

bool vehicle::getLaneChangingDecision(vehicle* o, vehicle* n, vehicle* q, vehicle* s, int targetLaneletId) {
    if(targetLaneletId == this->getNextLaneletId()) return true;
    if (this->acceleration_ < 0) return false; // We do not change lane during deccelerating
    // float ac_tilde = IDMAcceleration(s);
    // float ac = IDMAcceleration(q);
    // float ao_tilde;
    // float ao;
    // if(o == NULL) {
    //     ao_tilde = MAX_ACC;
    //     ao = MAX_ACC;
    // } else {
    //     ao_tilde = o->IDMAcceleration(q);
    //     ao = o->acceleration();
    // }
    // float an_tilde;
    // float an;
    // if(n == NULL) {
    //     an_tilde = MAX_ACC;
    //     an = MAX_ACC;
    // } else {
    //     an_tilde = n->IDMAcceleration(q);
    //     float an = n->acceleration();
    // }
    // float ac_eur_tilde = convertByEuropeRules(ac_tilde, n);
    // float ac_eur = convertByEuropeRules(ac, n);

    // // TODO    

    // // if (n && n->traveled_dis_ + MIN_DISTANCE > this->traveled_dis_) {
    // //     return false;
    // // }

    // // if (s && s->traveled_dis_ - MIN_DISTANCE < this->traveled_dis_) {
    // //     return false;
    // // }

    // // if ( (n && n->traveled_dis_ + MIN_DISTANCE > this->traveled_dis_) ||
    // //     && ( s || s->traveled_dis_ - MIN_DISTANCE < this->traveled_dis_))
    // //     return false;                   // To ensure an acceptable gap

    // return ((ac_eur_tilde - ac + POLITENESS * (an_tilde - an + ao_tilde - ao)) > CHANGEING_THRESHOLD)?
            // true : false;

}



