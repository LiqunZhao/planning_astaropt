//
//  lanelet.hpp
//  lanelet
//
//  Created by Ao LI on 25/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#ifndef lanelet_hpp
#define lanelet_hpp

#define MAX_ROAD_LENGTH 8888

#include <cstdio>
#include <vector>
#include <utility>
#include "vehicle.hpp"

class lanelet {
    
public:
    lanelet();
    ~lanelet();
    
    void init(int id, float max_len);
    void addInflowLanelet(lanelet* m, float dis);
    void addOutflowLanelet(lanelet* m, float dis);
    void vehicleGenerator();
    void addVehicle(vehicle* v);
    void removeVehicle(vehicle* v);
    void move2NextLaneletIfNeeded(vehicle* v);
    vehicle* findClosestPrecByLane(vehicle* v, lane_line lane);
    vehicle* findClosestSuccByLane(vehicle* v, lane_line lane);
    void updataAllVehicles();
    
private:
    std::vector<vehicle*> vehicle_vec_;
    std::pair<lanelet*, float> inflow_lanelet_; // The second key means the linked access position, TO REMOVE ? ~~~
    std::pair<lanelet*, float> outflow_lanelet_;
    
    int id_;
    int vehicle_num_;
    float max_road_length_;
};

#endif /* lanelet_hpp */
