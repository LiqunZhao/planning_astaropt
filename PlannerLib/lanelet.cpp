//
//  lanelet.cpp
//  lanelet
//
//  Created by Ao LI on 25/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#include <cmath>
#include <cstdlib>
#include "lanelet.hpp"

lanelet::lanelet(){
    this->vehicle_num_ = 0;
};

void lanelet::init(int id, float max_len){
    this->id_ = id;
    this->vehicle_num_ = 0;
    this->max_road_length_ = max_len;
};
void lanelet::addOutflowLanelet(lanelet* m, float dis){
    this->outflow_lanelet_.first = m;
    this->outflow_lanelet_.second = dis;
}

void lanelet::addInflowLanelet(lanelet* m, float dis){
    this->inflow_lanelet_.first = m;
    this->inflow_lanelet_.second = dis;
//    if (dis != 0) {
//        vehicle* virtual_vehicle = new vehicle(-1, 0, 0, 0, 0, lane_line::right, dis, 0);
//        this->addVehicle(virtual_vehicle);
//    }

}

void lanelet::vehicleGenerator() {
    float random = rand() % 30;
    float desired_velocity = 10 * (1 + random/100.0);
    float init_velocity = 1 * (1 + random/100.0);
    float init_lng_pos = 0;
    float init_lat_pos = 0;
    float init_acc = 0.2;
    int init_lane = rand() % 2;

    init_lat_pos = init_lane;
    
    int id = rand();
// id_(id), type_(t), velocity_(v), desired_velocity_(d_v), acceleration_(a), current_lane_(lane), lng_pos_(l_position)
    vehicle* v = new vehicle(id, 0, init_velocity, desired_velocity, init_acc, static_cast<lane_line>(init_lane), init_lng_pos, init_lat_pos, this->max_road_length_);
    v->calControlSolution();
    v->successor_ = NULL;
    
    vehicle* prec = findClosestPrecByLane(v, v->current_lane_);
    if(prec && prec->lng_pos_ - v->lng_pos_ < MIN_DISTANCE) {
        return;     // The precedent car is too close, cannot generate a new one
    }
    addVehicle(v);
    if (prec == NULL) {
        v->precessor_ = NULL;
        return;
    } else {
        v->precessor_ = prec;
        prec->successor_ = v;
    }
}

void lanelet::addVehicle(vehicle* v) {
    this->vehicle_num_++;
    this->vehicle_vec_.push_back(v);
}

void lanelet::removeVehicle(vehicle* v) {
    for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
        vehicle* tmp = *iter;
        if (tmp->id_ == v->id_){
            vehicle_vec_.erase(iter);
            this->vehicle_num_--;
        }
    }
}

vehicle* lanelet::findClosestPrecByLane(vehicle* v, lane_line lane) {
    float min_d = MAX_ROAD_LENGTH;
    vehicle* res = NULL;
    for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
        vehicle* tmp = *iter;
        float d = sqrt(pow(tmp->lng_pos_ - v->lng_pos_, 2) + pow(tmp->lng_pos_ - v->lng_pos_, 2));
        
        // Verify if the vehicle v is ahead of the found
        bool isVechicleBehind = false;
        if (v->lng_pos_ < tmp->lng_pos_) {
            isVechicleBehind = true;
        }
        
        if ((d < min_d) && isVechicleBehind && (tmp->current_lane_ == lane) && (tmp->id_ != v->id_)) {
            res = *iter;
            min_d = d;
        }
    }
    return res;
}

vehicle* lanelet::findClosestSuccByLane(vehicle* v, lane_line lane) {
    float min_d = MAX_ROAD_LENGTH;
    vehicle* res = NULL;
    for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
        vehicle* tmp = *iter;
        float d = sqrt(pow(tmp->lng_pos_ - v->lng_pos_, 2) + pow(tmp->lat_pos_ - v->lat_pos_, 2));
        
        // Verify if the vehicle v is ahead of the found
        bool isVechicleAheaded = false;
        if (v->lng_pos_ > tmp->lng_pos_) {
            isVechicleAheaded = true;
        }
        
        if ((d < min_d) && isVechicleAheaded && (tmp->current_lane_ == lane) && (tmp->id_ != v->id_)) {
            res = tmp;
            min_d = d;
        }
    }
    return res;
}

/*
 This function check if the car reaches the end of the road. If yes, move the car to the next lanelet
 */
void lanelet::move2NextLaneletIfNeeded(vehicle* v) {
    if ((v->lng_pos_ >= this->max_road_length_) && this->outflow_lanelet_.first) {
        if (this->outflow_lanelet_.second != 0) {
            v->current_lane_ = right;
        }
        v->lng_pos_ = this->outflow_lanelet_.second;
        v->setDestinationDistance(this->max_road_length_);
        v->calControlSolution();
        this->outflow_lanelet_.first->addVehicle(v);
        
        vehicle* np = this->outflow_lanelet_.first->findClosestPrecByLane(v, v->current_lane_);
        vehicle* ns = this->outflow_lanelet_.first->findClosestSuccByLane(v, v->current_lane_);
        
        vehicle* os = v->successor_;
        if (os) {
            os->precessor_ = NULL;
        }
        if (np) {
            v->precessor_ = np;
            np->successor_ = v;
        }
        if (ns) {
            v->successor_ = ns;
            ns->precessor_ = v;
        }
        this->removeVehicle(v);
    }
}

void lanelet::updataAllVehicles() {
    for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
        vehicle* tmp = *iter;
        tmp->updateStatus();
        this->move2NextLaneletIfNeeded(tmp);

        std::cout << this->id_ << "," << tmp->lng_pos_ << "," << tmp->lat_pos_ << "," << tmp->id_ << std::endl;

        lane_line side_lane;
        if (tmp->current_lane_ == left) {
            side_lane = right;
        } else {
            side_lane = left;
        }
        vehicle* n = findClosestSuccByLane(tmp, side_lane);
        vehicle* s = findClosestPrecByLane(tmp, side_lane);
        vehicle* o = tmp->successor_;
        vehicle* q = tmp->precessor_;
        if (!(n && s && o && q))
            continue;
        
        if ((tmp->lng_pos_ + MIN_DISTANCE < s->lng_pos_) &&
            (tmp->lng_pos_ - MIN_DISTANCE > n->lng_pos_) &&
             tmp->getLaneChangingDecision(n, s)) {
            tmp->changeLane(tmp->successor_, n, tmp->precessor_, s);
        }
    }
}
