//
//  server.cpp
//  server
//
//  Created by Ao LI on 25/04/2019.
//  Copyright © 2019 Ao LI. All rights reserved.
//

#include <cmath>
#include <cstdlib>
#include "server.hpp"

namespace {
std::string exampleMapPath = "/home/ao/Downloads/ElCerritoMap6.osm";
}  // namespace

server::server(){
    this->vehicle_num_ = 0;
};

void server::init(){
    using namespace lanelet;
    vehicle_num_ = 0;
    map_ = load(exampleMapPath, projection::UtmProjector(Origin({37.90046878383, -122.30342030542})));
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    routingGraph_  = routing::RoutingGraph::build(*map_, *trafficRules);
};

void server::generateShortestPathForVehicle(vehicle* v, int laneletId, int toLaneletId) {
    ConstLanelet lanelet = map_->laneletLayer.get(laneletId);
    ConstLanelet toLanelet = map_->laneletLayer.get(toLaneletId);
    Optional<routing::Route> route = routingGraph_->getRoute(lanelet, toLanelet, 0);
 
    routing::LaneletPath shortestPath = route->shortestPath();
    
    v->setShortestPath(shortestPath);
    v->setRoutingLanelet(lanelet, toLanelet);
}

void server::generateVechicleWithRoute(int laneletId, int toLaneletId) {
    float random = rand() % 30;
    float desired_velocity = 10 * (1 + random/100.0);
    float init_velocity = 1 * (1 + random/100.0);
    float init_lanelet_pos = 0;
    float init_acc = 0.2;
    int initLanelet = laneletId;
    int id = rand()%100;

    vehicle* v = new vehicle(id, 0, init_velocity, desired_velocity, init_acc, initLanelet, init_lanelet_pos);
    ConstLanelet currlanelet = map_->laneletLayer.get(v->currentLaneletId());
    auto centerline2d = lanelet::utils::to2D(currlanelet.centerline());
    BasicPoint2d pAt = geometry::interpolatedPointAtDistance(centerline2d, v->traveledDistanceInCurrLanelet());
    v->setPosition(pAt);
    addVehicle(v);
    vehicle* prec = findClosestPrecByLane(v, v->currentLaneletId());
    generateShortestPathForVehicle(v, laneletId, toLaneletId);
    if (prec == NULL) {
        v->setPrecessor(NULL);
        return;
    } else {
        v->setPrecessor(prec);
        prec->setSuccessor(v);
    }
}

void server::addVehicle(vehicle* v) {
    this->vehicle_num_++;
    this->vehicle_vec_.push_back(v);
}

void server::removeVehicle(vehicle* v) {
    for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
        vehicle* tmp = *iter;
        if (tmp->id() == v->id()){
            vehicle* prec = tmp->getPreccessor();
            vehicle* succ = tmp->getSuccessor();
            if(prec && succ) {
                prec->setSuccessor(succ);
                succ->setPrecessor(prec);
            } else if (prec) {
                prec->setSuccessor(NULL);
            } else if (succ) {
                succ->setPrecessor(NULL);
            }

            vehicle_vec_.erase(iter);                      
            this->vehicle_num_--;
            return;
        }
    }
}

vehicle* server::findClosestPrecByLane(vehicle* v, int laneId) {
    float min_d = std::numeric_limits<float>::max();
    ConstLanelet currLanelet = map_->laneletLayer.get(laneId);
    vehicle* res = NULL;
    ConstLanelets followingLanelets = routingGraph_->following(currLanelet);
    int depth = 0;
    while(!res && !followingLanelets.empty() && depth++<5) {
        for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
            vehicle* tmp = *iter;
            if (tmp->currentLaneletId() == currLanelet.id()) { //TODO
                BasicPoint2d pDiff = tmp->position() - v->position();
                double distPos = pDiff.transpose() * pDiff;

                if (v->currentLaneletId() == tmp->currentLaneletId() && v->traveledDistanceInCurrLanelet() > tmp->traveledDistanceInCurrLanelet())
                    continue;

                if ((distPos < min_d && tmp->id()!=v->id())){  //&& isVechicleBehind && (tmp->current_lane_ == lane) && (tmp->id_ != v->id_)) {
                    res = *iter;
                    min_d = distPos;
                }
            }
        }

        followingLanelets = routingGraph_->following(currLanelet);
        if (!followingLanelets.empty())
            currLanelet = *followingLanelets.begin();
    }
    if(!res) return NULL;
    if (followingLanelets.empty()) return NULL; 
    if (res->id() == v->id()) {return NULL;}
    // if (res->currentLaneletId() == v->currentLaneletId ||)  TO DO 

    return res;
}

vehicle* server::findClosestSuccByLane(vehicle* v, int laneId) {
    float min_d = std::numeric_limits<float>::max();
    ConstLanelet currLanelet = map_->laneletLayer.get(laneId);
    vehicle* res = NULL;
    ConstLanelets previousLanelets = routingGraph_->previous(currLanelet);
    
    int depth = 0;
    while(!res && !previousLanelets.empty() && depth++<5) {
        for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
            vehicle* tmp = *iter;
            if (tmp->currentLaneletId() == currLanelet.id()) { //TODO
                BasicPoint2d pDiff = tmp->position() - v->position();
                double distPos = pDiff.transpose() * pDiff;

                if (v->currentLaneletId() == tmp->currentLaneletId() && v->traveledDistanceInCurrLanelet() < tmp->traveledDistanceInCurrLanelet())
                    continue;

                if ((distPos < min_d && tmp->id()!=v->id())){  //&& isVechicleBehind && (tmp->current_lane_ == lane) && (tmp->id_ != v->id_)) {
                    res = *iter;
                    min_d = distPos;
                }
            }
        }

        previousLanelets = routingGraph_->previous(currLanelet);
        if (!previousLanelets.empty())
            currLanelet = *previousLanelets.begin();
    }
    if(!res) return NULL;
    if (previousLanelets.empty()) return NULL; 
    if (res->id() == v->id()) {return NULL;}
    // if (res->currentLaneletId() == v->currentLaneletId ||)  TO DO 

    return res;
}

//  This function check if the car reaches the end of a lanelet. If yes, move the car to the next lanelet
 
bool server::move2NextLanelet(vehicle* v, float dis) {
    v->setTraveledDistanceInCurrLanelet(dis);
    v->setLaneletId(v->getNextLaneletId());

    vehicle* newPrec = this->findClosestPrecByLane(v, v->currentLaneletId());
    vehicle* newSucc = this->findClosestSuccByLane(v, v->currentLaneletId());
    v->setPrecessor(newPrec);
    v->setSuccessor(newSucc);
    return false;
}

void server::vehicleLaneChangingIfNeeded(vehicle* v, ConstLanelet& targetLanelet) {
    vehicle* n = findClosestSuccByLane(v, targetLanelet.id());
    vehicle* s = findClosestPrecByLane(v, targetLanelet.id());
    vehicle* o = v->getSuccessor();
    vehicle* q = v->getPreccessor();

    if (v->getLaneChangingDecision(o, n, q, s, targetLanelet.id())) {
        Optional<routing::Route> route = routingGraph_->getRoute(v->startLanelet(), v->destinationLanelet(), 0);
        if (route->laneletMap()->laneletLayer.exists(targetLanelet.id())) {
            this->generateShortestPathForVehicle(v, targetLanelet.id(), v->destinationLanelet().id());
            /////
            ConstLanelet currLanelet = map_->laneletLayer.get(v->currentLaneletId());
            ConstLanelet targetLanelet = map_->laneletLayer.get(v->currentLaneletId());
            auto curCenterline2d = lanelet::utils::to2D(currLanelet.centerline());
            auto tarCenterline2d = lanelet::utils::to2D(currLanelet.centerline());
            v->a = geometry::interpolatedPointAtDistance(curCenterline2d, v->traveledDistanceInCurrLanelet());
            v->b = geometry::interpolatedPointAtDistance(curCenterline2d, v->traveledDistanceInCurrLanelet()+1.5);
            v->c = geometry::interpolatedPointAtDistance(tarCenterline2d, v->traveledDistanceInCurrLanelet()+1.5);
            v->d = geometry::interpolatedPointAtDistance(tarCenterline2d, v->traveledDistanceInCurrLanelet()+3);
            v->setChanging();
            /////
            v->changeLane(o, n, q, s, targetLanelet.id(), v->traveledDistanceInCurrLanelet());  // TODO new dis
        }
    }
}

void server::updateAllVehicles() {
    for (std::vector<vehicle*>::iterator iter = vehicle_vec_.begin(); iter != vehicle_vec_.end(); iter++) {
        vehicle* tmp = *iter;
        ////
        if (tmp->isChanging()) {
            tmp->updateStatus();
            continue;
        }
        ////

        ConstLanelet currlanelet = map_->laneletLayer.get(tmp->currentLaneletId());
        //////
        Optional<ConstLanelet> leftLanelet = routingGraph_->left(currlanelet);
        Optional<ConstLanelet> rightLanelet = routingGraph_->right(currlanelet);
        if (leftLanelet) {
            if(leftLanelet->id() == tmp->getNextLaneletId()) {
            move2NextLanelet(tmp, tmp->traveledDistanceInCurrLanelet());
            } 
        }
        if (rightLanelet) {
            if (rightLanelet->id() == tmp->getNextLaneletId()){
            move2NextLanelet(tmp, tmp->traveledDistanceInCurrLanelet());
            }
        }
        //////
        currlanelet = map_->laneletLayer.get(tmp->currentLaneletId());
        auto centerline2d = lanelet::utils::to2D(currlanelet.centerline());
        BasicPoint2d pBefore = geometry::interpolatedPointAtDistance(centerline2d, tmp->traveledDistanceInCurrLanelet());
        tmp->updateStatus();
        float disOffset = tmp->traveledDistanceInCurrLanelet() - geometry::approximatedLength2d(currlanelet);
        if (disOffset > 0) {
            if (tmp->destinationLanelet().id() == tmp->currentLaneletId()) {
                removeVehicle(tmp);
                iter--;
                continue;
            } else {
                this->move2NextLanelet(tmp, disOffset);
            }
        }

        currlanelet = map_->laneletLayer.get(tmp->currentLaneletId());
        centerline2d = lanelet::utils::to2D(currlanelet.centerline());
        BasicPoint2d pAfter = geometry::interpolatedPointAtDistance(centerline2d, tmp->traveledDistanceInCurrLanelet());
        BasicPoint2d pDirection = pAfter - pBefore;
        tmp->setHeading(std::atan2(pDirection.y(), pDirection.x()));
        tmp->setPosition(pAfter);

        // if (tmp->getShortestPath().size() <= 2) {
        //     continue;  // No need to change lane
        // }
        // leftLanelet = routingGraph_->left(currlanelet);
        // rightLanelet = routingGraph_->right(currlanelet);
        // if (leftLanelet) {
        //     vehicleLaneChangingIfNeeded(tmp, *leftLanelet);
        // } 
        // if (rightLanelet) {
        //     vehicleLaneChangingIfNeeded(tmp, *rightLanelet);
        // }
    }
}
