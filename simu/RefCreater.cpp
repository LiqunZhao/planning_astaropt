//
// Created by lcr on 7/25/19.
//

#include "RefCreater.h"

void RefCreater::Creater_init(int lanelet_begin_ID, int to_lanelet_ID, LaneletMapPtr map){
    lanelet_begin_ID_ = lanelet_begin_ID;
    to_lanelet_ID_ = to_lanelet_ID;
    map_ = map;
    LineString2d reference(utils::getId());
    reference_ = reference;
    Ref_generate();
}

void  RefCreater::Ref_generate(){
    traffic_rules::TrafficRulesPtr trafficRules =
            traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map_, *trafficRules);

    ConstLanelet lanelet = map_->laneletLayer.get(lanelet_begin_ID_);
    ConstLanelet toLanelet = map_->laneletLayer.get(to_lanelet_ID_);
    Optional<routing::Route> route = routingGraph->getRoute(lanelet, toLanelet, 0);
    routing::LaneletPath shortestPath = route->shortestPath();

    std::cout<<"Path size: "<<shortestPath.size()<<std::endl;

    for (auto ll : shortestPath){
        std::cout<< "ID: "<< ll.id()<<std::endl;
    }

    alglib::real_1d_array y_ref;
    alglib::real_1d_array x_ref;
    alglib::real_1d_array s_ref;
    y_ref.setlength(shortestPath.size()+1);
    x_ref.setlength(shortestPath.size()+1);
    s_ref.setlength(shortestPath.size()+1);
    double s_record[shortestPath.size()+1];
    s_record[0] = 0;

    auto centerline2d = lanelet::utils::to2D(shortestPath[0].centerline());
    BasicPoint2d pAt = geometry::interpolatedPointAtDistance(centerline2d, 0);
    std::cout<< "pat: "<< pAt(0,0) << pAt(1,0) <<std::endl;
    x_ref[0]= pAt(0,0);
    y_ref[0]= pAt(1,0);

    for (int j = 1; j < shortestPath.size(); j++) {
        centerline2d = lanelet::utils::to2D(shortestPath[j].centerline());
        pAt = geometry::interpolatedPointAtDistance(centerline2d, 0);
        std::cout<< "pat: "<< pAt(0,0) << pAt(1,0) <<std::endl;
        x_ref[j]= pAt(0,0);
        y_ref[j]= pAt(1,0);

        centerline2d = lanelet::utils::to2D(shortestPath[j-1].centerline());
        ArcCoordinates pAtt = geometry::toArcCoordinates(centerline2d, pAt);
        s_record[j] = pAtt.length;
    }
    centerline2d = lanelet::utils::to2D(shortestPath[shortestPath.size()-1].centerline());
    pAt = geometry::interpolatedPointAtDistance(centerline2d, 10);
    std::cout<< "pat: "<< pAt(0,0) << pAt(1,0) <<std::endl;
    x_ref[shortestPath.size()]= pAt(0,0);
    y_ref[shortestPath.size()]= pAt(1,0);
    s_record[shortestPath.size()] = 10;
    s_ref[0] = 0;
    for (int i = 1; i < shortestPath.size()+1; i++) {
        s_ref[i] = s_ref[i-1] + s_record[i];
        std::cout<< "s_ref: "<< s_ref[i] << " s_record: "<< s_record[i] << std::endl;
    }
    alglib::spline1dbuildcubic(s_ref, x_ref, spl_ref_xs_);
    alglib::spline1dbuildcubic(s_ref, y_ref, spl_ref_ys_);
    /*
    for (int i = 1; i < 700; i++){
        double xx = 0;
        double yy = 0;
        double dx = 0;
        double dy = 0;
        double d2x = 0;
        double d2y = 0;
        alglib::spline1ddiff(spl_ref_xs, 0.1 * i, xx, dx, d2x);
        alglib::spline1ddiff(spl_ref_ys, 0.1 * i, yy, dy, d2y);
        std::cout<<"2,"<< i <<","<< 100*i <<",car,"<< xx << ","<< yy<<",6.5473,0.266,0.0406,4.67,1.74"<< std::endl;
    }
    */
    y_ref.setlength(690000);
    x_ref.setlength(690000);
    s_ref.setlength(690000);
    s_ref[0]=0;
    for (int i = 0; i < 690000;i++){
        double xx = 0;
        double yy = 0;
        double dx = 0;
        double dy = 0;
        double d2x = 0;
        double d2y = 0;
        alglib::spline1ddiff(spl_ref_xs_, i*0.0001, xx, dx, d2x);
        alglib::spline1ddiff(spl_ref_ys_, i*0.0001, yy, dy, d2y);
        x_ref[i] = xx;
        y_ref[i] = yy;
        Point2d p(utils::getId(),xx,yy);
        reference_.push_back(p);
        if (i>0) s_ref[i] = s_ref[i-1] + sqrt((xx-x_ref[i-1])*(xx-x_ref[i-1])+(yy-y_ref[i-1])*(yy-y_ref[i-1]));
    }
    alglib::spline1dbuildcubic(s_ref, x_ref, spl_ref_xs_);
    alglib::spline1dbuildcubic(s_ref, y_ref, spl_ref_ys_);

}


alglib::spline1dinterpolant RefCreater::getXS(){
    return spl_ref_xs_;
}
alglib::spline1dinterpolant RefCreater::getYS(){

    return spl_ref_ys_;
}
LineString2d RefCreater::getRef(){
    return reference_;
}