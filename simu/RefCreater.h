//
// Created by lcr on 7/25/19.
//

#ifndef LANELET2_REFCREATER_H
#define LANELET2_REFCREATER_H
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include "../alglib/stdafx.h"
#include "../alglib/interpolation.h"


using namespace lanelet;

class RefCreater {
public:

    void Creater_init(int lanelet_begin_ID, int to_lanelet_ID, LaneletMapPtr map);
    void Ref_generate();
    alglib::spline1dinterpolant getXS();
    alglib::spline1dinterpolant getYS();
    LineString2d getRef();

private:
    LaneletMapPtr map_;
    int lanelet_begin_ID_;
    int to_lanelet_ID_;
    alglib::spline1dinterpolant spl_ref_xs_;
    alglib::spline1dinterpolant spl_ref_ys_;
    LineString2d reference_;

};


#endif //LANELET2_REFCREATER_H
