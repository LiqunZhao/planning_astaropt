//
//  server.hpp
//  server
//
//  Created by Ao LI on 25/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#ifndef server_hpp
#define server_hpp

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

#include "vehicle.hpp"

class server {
    
public:
    server();
    ~server();
    
    void init();
    void generateShortestPathForVehicle(vehicle* v, int laneletId, int toLaneletId);
    void generateVechicleWithRoute(int laneletId, int toLaneletId);
    void addVehicle(vehicle* v);
    void removeVehicle(vehicle* v);
    bool move2NextLanelet(vehicle* v, float dis);
    vehicle* findClosestPrecByLane(vehicle* v, int laneId);
    vehicle* findClosestSuccByLane(vehicle* v, int laneId);
    void vehicleLaneChangingIfNeeded(vehicle* v, ConstLanelet& targetLanelet);
    void updateAllVehicles();
    
private:
    std::unique_ptr<LaneletMap> map_;
    std::unique_ptr<routing::RoutingGraph> routingGraph_;
    // routing::Route route_;
    std::vector<vehicle*> vehicle_vec_;
    std::vector<vehicle*> vehiclesNeedToRemove_;
    int vehicle_num_;
};

#endif /* server_hpp */
