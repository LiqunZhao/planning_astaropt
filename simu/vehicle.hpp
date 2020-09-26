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
#define MAX_DECC -2
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

#include <cstdio>
#include <vector>
#include <utility>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/LaneletMap.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Primitive.h>

#include <lanelet2_matching/LaneletMatching.h>

using namespace lanelet;
using namespace lanelet::matching;

/*
 * OVM parameters
 */
const double L = 20;      // System Size
const double C = 2.0;

// end of OVM parameters

class vehicle {

public:
    vehicle(int id, int t, float v, float d_v, float a, int laneId, float traveledDistance):
            id_(id), type_(t), velocity_(v), desired_velocity_(d_v), acceleration_(a), currentLaneletId_(laneId), traveled_dis_(traveledDistance) {
                successor_ = NULL;
            }
    ~vehicle();

    void setSuccessor(vehicle* succ);
    void setPrecessor(vehicle* prec);
    void setTraveledDistanceInCurrLanelet(float dis);
    void setLaneletId(int laneletId);
    void setChanging() {is_changing_ = true;};
    vehicle* getSuccessor() {return successor_;};
    vehicle* getPreccessor() {return precessor_;};
    routing::LaneletPath getShortestPath() {return shortestPath_;};
    
    void setPosition(BasicPoint2d& position) {position_= position;};
    void setHeading(float heading) {heading_ = heading;};
    void setRoutingLanelet(ConstLanelet& startLanelet, ConstLanelet& destinationLanelet);
    void setShortestPath(const routing::LaneletPath& shortestPath);
    int getNextLaneletId();

    void setDestinationDistance(float dis);
    void changeLane(vehicle* o, vehicle* n, vehicle* q, vehicle* s, int newLaneletId, float newDis);
    void updateStatus();
    void updateStatusWithReferenceLine(int id);
    float IDMAcceleration(vehicle* prec);

    inline double V(double dx){ return tanh(dx - C)+tanh(C);};

    float OVMAcceleration(vehicle* prec);
    float GFMAcceleration(vehicle* prec);
    float convertByEuropeRules(float a_tilde, vehicle* prec);
    bool getLaneChangingDecision(vehicle* o, vehicle* n, vehicle* q, vehicle* s, int targetLaneletId);

    int id() {return id_;};
    int currentLaneletId() {return currentLaneletId_;};
    ConstLanelet startLanelet() {return startLanelet_;};
    ConstLanelet destinationLanelet() {return destinationLanelet_;};
    BasicPoint2d& position() { return position_;};
    BasicPoint2d& targetPosition() { return targetPosition_;};
    float heading() {return heading_;};
    float velocity() {return velocity_;};
    float acceleration() {return acceleration_;};
    float traveledDistanceInCurrLanelet() {return traveled_dis_;};
    bool isChanging() {return is_changing_;};

//////
    BasicPoint2d a;
    BasicPoint2d b;
    BasicPoint2d c;
    BasicPoint2d d;
    float t = 0.5f;
    
 private:
    int id_;
    int type_;   //TODO use enum now 0 for car, 1 for truck
    float velocity_;
    float desired_velocity_;
    float acceleration_;

    int currentLaneletId_;
    int startLaneletId_;
    int destinationLaneletId_;

    float traveled_dis_;

    routing::LaneletPath shortestPath_;
    // Object2d position_;
    BasicPoint2d position_;
    BasicPoint2d targetPosition_;
    float heading_;

    ConstLanelet startLanelet_;
    ConstLanelet destinationLanelet_;
    
    bool is_changing_ = false; // -1 means the car is changing from left to right, 1 means the car is changing from right to left

    vehicle* successor_;
    vehicle* precessor_;


};

#endif /* vehicle_hpp */
