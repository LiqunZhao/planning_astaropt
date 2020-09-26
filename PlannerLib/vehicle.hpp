//
//  vehicle.hpp
//  MOBIL
//
//  Created by Ao LI on 26/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#define POLITENESS 0
#define CHANGEING_THRESHOLD 0.1
#define MAX_SAFE_DECELERATION 4
#define BIAS_FOR_RIHGT_LANE 0.3
#define MAX_ACC 1.2
#define DESIRED_DECC -2
#define TIME_GAP 0.3
#define MIN_DISTANCE 8
#define CRITICAL_VELOCITY 180

#define wA
#define wT


#ifndef vehicle_hpp
#define vehicle_hpp

#include <stdio.h>
#include <iostream>
#include <cmath>


enum  lane_line {
    left = 0,
    right,
}; // TODO add more lanes

class vehicle {
    
public:
    vehicle(int id, int t, float v, float d_v, float a, lane_line lane, float lng_position, float lat_position, float dest_dis):
            id_(id), type_(t), velocity_(v), desired_velocity_(d_v), acceleration_(a), current_lane_(lane), lng_pos_(lng_position), lat_pos_(lat_position), sig_value_(static_cast<float>(lane)), wA_(wA), wT_(wT), dest_dis_(dest_dis), jerk_(0), time_count_(0), is_changing_(0) {}
    ~vehicle();
    void calControlSolution();
    void calCoefficients();
    float findOptimalTime();   // call this function in the very beginning
    
    void setSuccessor(vehicle* succ);
    void setPrecessor(vehicle* prec);
    void setDestinationDistance(float dis);
    void changeLane(vehicle* o, vehicle* n, vehicle* q, vehicle* s);
    void updateStatus();
    void updateStatusWithReferenceLine(int id);
    float predictAcceleration(vehicle* prec);
    float convertByEuropeRules(float a_tilde, vehicle* prec);
    bool getLaneChangingDecision(vehicle* n , vehicle* s);
    
    float lng_pos_;
    float lat_pos_;
    float heading_;
    
// private:
    int id_;
    int type_;   //TODO use enum now 0 for car, 1 for truck
    float velocity_;
    float desired_velocity_;
    float acceleration_;
    
    float jerk_;
    float wT_;
    float wA_;
    float dest_dis_;    // Also noted as sf
    int time_count_;
    float sig_value_; // This variable is for smoothing the process of lane change
    
    lane_line current_lane_;
    
    int is_changing_; // -1 means the car is changing from left to right, 1 means the car is changing from right to left

    vehicle* successor_;
    vehicle* precessor_;
    
    float c1;
    float c2;
    float c3;
    float c4;
    float c5;
};

#endif /* vehicle_hpp */
