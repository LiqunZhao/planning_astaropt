//
// Created by Chenran on 06/10/19.
//


#include "IDM.hpp"

IDM::IDM(double current_v, double lng_position, double desire_v, double front_lng_psition, double front_v):
        velocity_(current_v), desired_velocity_(desire_v), lng_pos_(lng_position), front_lng_pos_(front_lng_psition), front_velocity_(front_v){

    double s_star = MIN_DISTANCE + velocity_ * TIME_GAP + front_velocity_ / (2 * sqrt(MAX_ACC * (-DESIRED_DECC)));
    double s_alpha = front_lng_pos_- lng_pos_;
    acceleration = MAX_ACC * (1 - pow((velocity_/desired_velocity_), 4) - pow(s_star/s_alpha, 2));
}



IDM::IDM(double current_v, double lng_position, double desire_v):
    velocity_(current_v), desired_velocity_(desire_v), lng_pos_(lng_position), front_lng_pos_(0), front_velocity_(0){
    acceleration = (MAX_ACC * (1 - pow((velocity_/desired_velocity_),4)));
}




std::vector<double> IDM::getstate(double t){
    std::vector<double> states;
    double next_lng_pos = lng_pos_ + velocity_ * t + 0.5*acceleration*t*t;
    double next_v = velocity_ + acceleration*t;
    double next_a = acceleration;
    states.push_back(next_lng_pos);
    states.push_back(next_v);
    states.push_back(next_a);
    return states;
}