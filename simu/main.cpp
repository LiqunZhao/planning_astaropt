//
// Created by zlq on 7/24/20.
//
#include "sys/time.h"
#include "txtRef.h"
#include "RefCreater.h"
#include "car.h"
#include "boost/multi_array.hpp"
#include "../PlannerLib/AstarSolver.h"
#include "../PlannerLib/QuadProg++.hh"
#include "../PlannerLib/QPSetup.h"
#include "../PlannerLib/smoother.h"
#include <lanelet2_matching/LaneletMatching.h>
#include <string>
#include <queue>
#include <time.h>
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <bitset>
#include <thread>
#include <mutex>
#include <cstdio>
#include<cstring>
#include<algorithm>
#include<cstdlib>
#include<streambuf>

using namespace lanelet;

/*int power(int a,int b)
{
	int val=1;
	for (int i = 0; i < b; i++)
	{
		val*=a;
	}
	return val;
}
 
int get_length(int a)
{
	int length=0;
	while(a)
	{
		a/=10;
		length++;
	}
	return length;
}
 
int get_value(int a,int i)
{
	int value;
	value=a%power(10,i)/power(10,i-1);
	return value;
}
*/


template <class Type>
Type stringToNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
} 
 
struct As_choice{
        int id;
        double s_f;
        double s_b;
        double t_i;
        double t_o;
        double v;

    };
 
static const float INF = 1000000;
static const float EPSLN = float(1e-3);
float w_a = 5;
float w_an = 5;
float w_v = 1;
float w_change = 0;
float w_runover = -100;
float desiredV = 3;
double V_s;
double DT = 1.0;                               // define the time step used by Astar algotithm              
double DDTT = 0.1;                             // define the real time step for the whole simulation, here we use 0.1s
double DTT = DDTT * 1000.00;                   // convert DDTT from second to ms
std::vector<int> actions = {-3,-2,-1,0,1,2,3}; //Define possible accelerations
int horizon = 5;
double curvature[12500];
vector<double> PATH1(5);                       // to store the actions decided in the previous loop
vector<double> AstarX;                         // to store the x calculated by Astar in the previous loop
vector<double> AstarY;                         // to store the y calculated by Astar in the previous loop
vector<double> D2X;                            // to store the x accelaration calculated in the previous loop
vector<double> D2Y;                            // to store the y accelaration calculated in the previous loop
vector<double> AstarVX;                        // to store the vx calculated by Astar in the previous loop
vector<double> AstarVY;                        // to store the vy calculated by Astar in the previous loop
vector<double> QPX;                            // to store the x calculated by QP with another method(to use ax and ay as initialization or simply 0) in the previous loop
vector<double> QPY;                            // to store the y calculated by QP with another method(to use ax and ay as initialization or simply 0) in the previous loop
vector<double> QPVX;                           // to store the vx calculated by QP with another method(to use ax and ay as initialization or simply 0) in the previous loop
vector<double> QPVY;                           // to store the vy calculated by QP with another method(to use ax and ay as initialization or simply 0) in the previous loop


 
std::mutex mutex_for_AS;
vector<Node> result_space;
std::vector<As_choice> As_ob_info;
 
 
 
 
 /*void As_cau(int i){
 	std::string binary = std::bitset<5>(i).to_string(); //to binary
    auto AstarSolver = SpeedProfAstarSolver(0.0, V_s, horizon, 1.0, actions,PATH1,i);//params: s0 (ego vehicle), v0 (ego vehicle), horizon, dt, actions
    AstarSolver.w_a_ = w_a;
    AstarSolver.w_an_ = w_an;
    AstarSolver.w_v_ = w_v;
    AstarSolver.desiredV_ = desiredV;
    AstarSolver.w_change_ = w_change;
    AstarSolver.w_runover_ = w_runover;
    AstarSolver.curvature_ = curvature;
 	for (int j = 0; j < As_ob_info.size(); j++){
 		if (binary[4-j] == '1'){
 			SpeedProfAstarSolver::Vehicle vehi_ro = SpeedProfAstarSolver::Vehicle(As_ob_info[j].s_b,0,As_ob_info[j].t_i);
            vehi_ro.run_over_ = true;
            AstarSolver.env_.Vehicles_.push_back(vehi_ro);
		 } else{
            SpeedProfAstarSolver::Vehicle vehi_bh = SpeedProfAstarSolver::Vehicle(As_ob_info[j].s_f,0,As_ob_info[j].t_o);
            vehi_bh.run_over_ = false;
            AstarSolver.env_.Vehicles_.push_back(vehi_bh);
        }
	 }
	 
	Node result = AstarSolver.plan();
    mutex_for_AS.lock();
    result_space.push_back(result);
    mutex_for_AS.unlock();
 }*/
 
 
 int main(){

    string formal_decision = "00000";	
    char write_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";
    timeval T_now;
    tm *area;
    gettimeofday(&T_now,NULL);
    area=localtime(&(T_now.tv_sec));
    sprintf(write_file_name,"../record/txt/test_%s.txt",asctime(area));
    ofstream File_creat(write_file_name);
    File_creat.close();
 	int youxiao = 0;                     //to calculate the number of loops containing information of obstacles
 	int zongshu = 0;                     //to calculate the number of loops in total
 	int xiangchao = 0;                   //to calculate the number of loops where host car meets obstacle(s) and decide to run over
 	string taibaoshou = "00000";


    D2X.push_back(0);                    
    D2Y.push_back(0);

 	
 	ifstream track_file("../record/new/15_1.3_0.1.csv", ios::in);                                // the document containing the information of environment
 	string str;
    getline(track_file, str);

    std::vector<car*> data_car;
    string track_ID, frame_ID, timestamp_ms, agent_type, x, y, vx, vy, psi_rad, length, width;
    string old_track_ID;
 	
 	getline(track_file, track_ID, ',');
    getline(track_file, frame_ID, ',');
    getline(track_file, timestamp_ms, ',');
    getline(track_file, agent_type, ',');
    getline(track_file, x, ',');
    getline(track_file, y, ',');
    getline(track_file, vx, ',');
    getline(track_file, vy, ',');
    getline(track_file, psi_rad, ',');
    getline(track_file, length, ',');
    getline(track_file, width, '\n');

    car* new_car = new class car;
    new_car->id_ = stringToNum<int>(track_ID);
    old_track_ID=track_ID;
    data_car.push_back(new_car);
    state reading_state;
    reading_state.time_ = stringToNum<int>(timestamp_ms);
    reading_state.length_ = stringToNum<double>(length);
    reading_state.width_ = stringToNum<double>(width);
    reading_state.psi_rad_ = stringToNum<double>(psi_rad);
    reading_state.vx_ = stringToNum<double>(vx);
    reading_state.vy_ = stringToNum<double>(vy);
    reading_state.x_ = stringToNum<double>(x);
    reading_state.y_ = stringToNum<double>(y);
    new_car->car_state.push_back(reading_state);

    while (getline(track_file, track_ID, ',')) {
        getline(track_file, frame_ID, ',');
        getline(track_file, timestamp_ms, ',');
        getline(track_file, agent_type, ',');
        getline(track_file, x, ',');
        getline(track_file, y, ',');
        getline(track_file, vx, ',');
        getline(track_file, vy, ',');
        getline(track_file, psi_rad, ',');
        getline(track_file, length, ',');
        getline(track_file, width, '\n');
        if (old_track_ID!=track_ID){
            new_car = new class car;
            new_car->id_ = stringToNum<int>(track_ID);
            old_track_ID=track_ID;
            data_car.push_back(new_car);
        }
        reading_state.time_ = stringToNum<int>(timestamp_ms);
        reading_state.length_ = stringToNum<double>(length);
        reading_state.width_ = stringToNum<double>(width);
        reading_state.psi_rad_ = stringToNum<double>(psi_rad);
        reading_state.vx_ = stringToNum<double>(vx);
        reading_state.vy_ = stringToNum<double>(vy);
        reading_state.x_ = stringToNum<double>(x);
        reading_state.y_ = stringToNum<double>(y);
        new_car->car_state.push_back(reading_state);
    }
	
	txtRef refcreater;
	string ref_file = "../ref1.txt";
	refcreater.Creater_init(ref_file);
	alglib::spline1dinterpolant spl_ref_xs;
    alglib::spline1dinterpolant spl_ref_ys;
    LineString2d reference;
    BasicPoint2d pAt;

    spl_ref_xs = refcreater.getXS();
    spl_ref_ys = refcreater.getYS();
    reference = refcreater.getRef();
    
    pAt = geometry::interpolatedPointAtDistance(reference, 10);
    car* mycar = new class car;
    mycar->id_ = 0;
    state inistate;
    inistate.time_ = 0;
    inistate.length_ = 4.67;                          // The initializations, can be changed accroding to specific situation
    inistate.width_ = 1.74;
    inistate.psi_rad_ = 0;
    inistate.vx_ = 2;
    inistate.vy_ = 0;
    inistate.x_ = -13;
    inistate.y_ = 0;
    mycar->car_state.push_back(inistate);

    AstarVX.push_back(inistate.vx_);
    AstarVY.push_back(0);                            // can be changed as AstarVY.push_back(inistate.vy_); 
    AstarX.push_back(inistate.x_);
    AstarY.push_back(0);                             // can be changed as AstarVY.push_back(inistate.y_); 
    QPVX.push_back(inistate.vx_);
    QPVY.push_back(0);                               // can be changed as AstarVY.push_back(inistate.vy_); 
    QPX.push_back(inistate.x_);
    QPY.push_back(0);                                // can be changed as AstarVY.push_back(inistate.y_); 
    
    int time_now = 0;
    
    time_t START = clock();
    while(true){
    	
    std::ofstream out;
	out.open(write_file_name,std::ios::app);	
	cout<<"/*************************************************************************/"<<endl;
    state mycar_state_now = mycar->getstate(time_now);
    out << "0,"<< mycar_state_now.time_/1000 <<","<< mycar_state_now.time_ <<",car, x: "<< mycar_state_now.x_ << ", y: "<< mycar_state_now.y_<<", vx: "<<mycar_state_now.vx_<<", vy: "<<mycar_state_now.vy_<<", psi_rad: "<<mycar_state_now.psi_rad_<<","<<"4.67,1.74"<< std::endl;
    BasicPoint2d Position_now(mycar_state_now.x_,mycar_state_now.y_);
    
    ArcCoordinates pAtt = geometry::toArcCoordinates(reference, Position_now);
    mycar->s_now = pAtt.length;
    std::cout<<"s_now: "<<mycar->s_now<<std::endl;
    if (mycar->s_now > 60) break;
    zongshu = zongshu + 1;
    
    double xx = 0;
    double yy = 0;
    double dx = 0;
    double dy = 0;
    double d2x = 0;
    double d2y = 0;
	
    alglib::spline1ddiff(spl_ref_xs, mycar->s_now, xx, dx, d2x);
    alglib::spline1ddiff(spl_ref_ys, mycar->s_now, yy, dy, d2y);
    double orientation = atan2(dy,dx);
    
    for (int i = 0; i < 12500; i++){
    	alglib::spline1ddiff(spl_ref_xs, mycar->s_now + 0.01*i, xx, dx, d2x);
        alglib::spline1ddiff(spl_ref_ys, mycar->s_now + 0.01*i, yy, dy, d2y);
        curvature[i] =abs(dx*d2y-dy*d2x)/pow(dy*dy+dx*dx,1.5);
	}
	
	V_s = sqrt(mycar_state_now.vx_*mycar_state_now.vx_+mycar_state_now.vy_*mycar_state_now.vy_);
	cout<<"current vs: "<<V_s<<endl;
    double LENGTHPATH = V_s*5+0.5*3*25;                                  
    cout<<"the longest possible path is: "<<LENGTHPATH<<endl;
	
	std::vector<As_choice> follow_info;
	
	for (auto ob:data_car){
		double s_f;
        double s_b;
        double t_i;
        double t_o;
        double V_for_f;
        bool init_flag = false;
        
		auto ob_state = ob->getstate(time_now);
		
		for (int i = 0; i < 51; i++){
			auto ob_state = ob->getstate(time_now + i*100);
			if(ob_state.time_ == time_now + i*100){
                                //cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                                //cout<<"ob_state.x_ is: "<<ob_state.x_<<"  ob_state.y_ is: "<<ob_state.y_<<endl;
                                //cout<<ob_state.y_ - (ob_state.length_)/2<<" "<<ob_state.y_ + (ob_state.length_)/2<<" "<<ob_state.x_ - (ob_state.width_)/2<<" "<<ob_state.x_ + (ob_state.width_)/2<<endl;
				bool ob_flag_f = false;
				bool ob_flag_b = false;
				for (int j = 0; j < (V_s*5+0.5*3*25)*100;j++){                                  // When the largest acceleration is 5
				alglib::spline1ddiff(spl_ref_xs, mycar->s_now + 0.01*j, xx, dx, d2x);
                                alglib::spline1ddiff(spl_ref_ys, mycar->s_now + 0.01*j, yy, dy, d2y);    
                                //cout<<"xx is: "<<xx<<" yy is: "<<yy<<endl;
                    if ((yy + ( mycar_state_now.width_ )/2> ob_state.y_ - (ob_state.length_)/2) and (yy - ( mycar_state_now.width_ )/2< ob_state.y_ + (ob_state.length_)/2) and (xx + (  mycar_state_now.length_ )/2> ob_state.x_ - (ob_state.width_)/2) and (xx - ( mycar_state_now.length_ )/2< ob_state.x_ + (ob_state.width_)/2)){
                        // the upper line of code could be changed according to the situation, this is a simple version for checking whether there is a collision between host car and obstacles
                    	if(!ob_flag_f){
                    		ob_flag_f = true;
                    		if(!init_flag){
                    			init_flag = true;
                    			s_f = 0.01*j;
                    		    s_b = s_f;
                    		    t_i = 0.1*i;
                    		    V_for_f = ob_state.vx_*cos(atan2(dy,dx))+ob_state.vy_*sin(atan2(dy,dx));	
							}else if(0.01*j<s_f)
							s_f = 0.01*j;
					    }
				    }
				    else if(ob_flag_f){
				    	if (0.01*j > s_b) s_b = 0.01*j;
				    	t_o = 0.1*i;
				    	ob_flag_b = true;
				    	break;
					}
			
			    }
			    if (ob_flag_f and !ob_flag_b) {
                                s_b = (V_s*5+0.5*3*25);
                            }
			
	     	}
		}
		
		if (init_flag){
            cout << ob->id_ << "  s_f " << s_f << "  s_b " << s_b << "  t_i " << t_i << "  t_o " << t_o << endl;
            out << ob->id_ << "  s_f " << s_f << "  s_b " << s_b << "  t_i " << t_i << "  t_o " << t_o << endl;
	    As_choice add_choice;
            add_choice.id = ob->id_;
            add_choice.s_f = s_f;
            add_choice.s_b = s_b;
            add_choice.t_i = t_i;
            add_choice.t_o = t_o;
            add_choice.v = V_for_f;
            if (s_b - s_f > 200) follow_info.push_back(add_choice);
            else As_ob_info.push_back(add_choice);
		}
	}
	
	vector<thread*> AS_thread;
	vector<string> run_over_decision;
	
	time_t start = clock();
	bool using_thread = false;

    if(!As_ob_info.empty()){
		auto AstarSolver = SpeedProfAstarSolver(0.0,V_s,horizon,1.0,actions,PATH1);
		AstarSolver.w_a_ = w_a;
        AstarSolver.w_an_ = w_an;
        AstarSolver.w_v_ = w_v;
        AstarSolver.w_change_ = w_change;
        AstarSolver.w_runover_ = w_runover;
        AstarSolver.desiredV_ = desiredV;
        AstarSolver.curvature_ = curvature;
        for (int j = 0; j < As_ob_info.size(); j++){
        	SpeedProfAstarSolver::Vehicle vehi_eff = SpeedProfAstarSolver::Vehicle(As_ob_info[j].s_f,As_ob_info[j].s_b,0,As_ob_info[j].t_i,As_ob_info[j].t_o);  //provide all the information instead of part of it
            //vehi_ro.run_over_ = true;
            AstarSolver.env_.Vehicles_.push_back(vehi_eff);
        }
        for(auto one_follow : follow_info){
			 	SpeedProfAstarSolver::Vehicle vehi_fo = SpeedProfAstarSolver::Vehicle(one_follow.s_f,one_follow.s_b,one_follow.v,one_follow.t_i,one_follow.t_o);
                vehi_fo.follow_ = true;
                AstarSolver.env_.Vehicles_.push_back(vehi_fo);
                cout<<"follow car id: "<<one_follow.id<<endl;
			 }
			 
			 result_space.push_back(AstarSolver.plan());
	}
	else{
		auto AstarSolver = SpeedProfAstarSolver(0.0, V_s, horizon, 1.0, actions,PATH1);
        AstarSolver.w_a_ = w_a;
        AstarSolver.w_an_ = w_an;
        AstarSolver.w_v_ = w_v;
        AstarSolver.w_change_ = w_change;
        AstarSolver.w_runover_ = w_runover;
        AstarSolver.desiredV_ = desiredV;
        AstarSolver.curvature_ = curvature;
        for (auto one_follow : follow_info) {
            SpeedProfAstarSolver::Vehicle vehi_fo = SpeedProfAstarSolver::Vehicle(one_follow.s_f,one_follow.s_b,one_follow.v,one_follow.t_i,one_follow.t_o);
            vehi_fo.follow_ = true;
            AstarSolver.env_.Vehicles_.push_back(vehi_fo);
            cout<<"follow car id: "<<one_follow.id<<endl;
        }
        result_space.push_back(AstarSolver.plan());
	}

	/*if(!As_ob_info.empty()) {
		for (int i = 0; i < pow(2,As_ob_info.size());i++){
			std::string binary = std::bitset<5>(i).to_string();
			run_over_decision.push_back(binary);
 	        auto AstarSolver = SpeedProfAstarSolver(0.0,V_s,horizon,1.0,actions,PATH1,i);
		 	AstarSolver.w_a_ = w_a;
            AstarSolver.w_an_ = w_an;
            AstarSolver.w_v_ = w_v;
            AstarSolver.w_change_ = w_change;
            AstarSolver.w_runover_ = w_runover;
            AstarSolver.desiredV_ = desiredV;
            AstarSolver.curvature_ = curvature;
		 	for (int j = 0; j < As_ob_info.size(); j++){
		 		if (binary[4-j] == '1'){
		 			SpeedProfAstarSolver::Vehicle vehi_ro = SpeedProfAstarSolver::Vehicle(As_ob_info[j].s_b,0,As_ob_info[j].t_i);
                    vehi_ro.run_over_ = true;
                    AstarSolver.env_.Vehicles_.push_back(vehi_ro);
				 }else{
				 	SpeedProfAstarSolver::Vehicle vehi_bh = SpeedProfAstarSolver::Vehicle(As_ob_info[j].s_f,0,As_ob_info[j].t_o);
                    vehi_bh.run_over_ = false;
                    AstarSolver.env_.Vehicles_.push_back(vehi_bh);
				 }
			 }
			 for(auto one_follow : follow_info){
			 	SpeedProfAstarSolver::Vehicle vehi_fo = SpeedProfAstarSolver::Vehicle(one_follow.s_f,one_follow.v,one_follow.t_i);
                vehi_fo.follow_ = true;
                AstarSolver.env_.Vehicles_.push_back(vehi_fo);
                cout<<"follow car id: "<<one_follow.id<<endl;
			 }
			 
			 result_space.push_back(AstarSolver.plan());
			
		}
		
	}else{
		auto AstarSolver = SpeedProfAstarSolver(0.0, V_s, horizon, 1.0, actions,PATH1,0);//params: s0 (ego vehicle), v0 (ego vehicle), horizon, dt, actions
        AstarSolver.w_a_ = w_a;
        AstarSolver.w_an_ = w_an;
        AstarSolver.w_v_ = w_v;
        AstarSolver.w_change_ = w_change;
        AstarSolver.w_runover_ = w_runover;
        AstarSolver.desiredV_ = desiredV;
        AstarSolver.curvature_ = curvature;
        for (auto one_follow : follow_info) {
            SpeedProfAstarSolver::Vehicle vehi_fo = SpeedProfAstarSolver::Vehicle(one_follow.s_f,one_follow.v,one_follow.t_i);
            vehi_fo.follow_ = true;
            AstarSolver.env_.Vehicles_.push_back(vehi_fo);
            cout<<"follow car id: "<<one_follow.id<<endl;
        }
        result_space.push_back(AstarSolver.plan());
	}*/
	if (using_thread){
        for(int i = 0;i<AS_thread.size();i++){
            AS_thread[i]->join();
            delete AS_thread[i];
        }
    }
    
    double min_cost = INF;
    //double formal_cost = -1;
    bool find_flag = false;
    bool find_final = false;
    Node node;
    //string final_decision;
    vector<int> final_decision(5,0);
    cout<<"The number of the result_space is: "<<result_space.size()<<endl;
    for (int i = 0; i < result_space.size(); i++){
        cout<<result_space[i].total_cost_<<endl;
    }

    for (int i = 0; i<result_space.size();i++){
           //cout<<"cau_node.total_cost_: "<<cau_node.total_cost_<<endl;
        if (result_space[i].total_cost_ < min_cost){
            min_cost = result_space[i].total_cost_;
            node.path_ = result_space[i].path_;
            node.total_cost_ = result_space[i].total_cost_;
            if (!As_ob_info.empty()) {
            	vector<int> PASSORNOT = result_space[i].Passornot_;
            	for (size_t i = 0;i<(PASSORNOT.size());i++){
            		if(PASSORNOT[i] == 1){
            			final_decision[i] = 0;                    //没超车
					}
					else if (PASSORNOT[i] == 3){
						final_decision[i] = 1;                   //超车
					}
                    else if (PASSORNOT[i] == 0){ 
                        final_decision[i] = 2;                  //行车不安全,return了INF
                    }
                    else if (PASSORNOT[i] == 2){
                        final_decision[i] = 3;                  //Astar部分没有处理，没有足够的obstacle
                    }
				}
            	 
                find_final = true;
	    }
            else cout<<"There is no element in As_ob_info"<<endl;
            find_flag = true;
        }
    }


    /*for(int i = 0; i < result_space.size();i++){
        cout<<"The paths in the result sapce are seperately: "<<endl;
        for (int j = 0; j < 5; j++){
            cout<<result_space[i].path_[j]<<" ";
        }
        cout<<endl;
    }

    for (int i = 0; i<result_space.size();i++){
           //cout<<"cau_node.total_cost_: "<<cau_node.total_cost_<<endl;
        if (result_space[i].total_cost_ < min_cost){
            min_cost = result_space[i].total_cost_;
            node.path_ = result_space[i].path_;
            node.total_cost_ = result_space[i].total_cost_;
            if (!As_ob_info.empty()) {
                final_decision = run_over_decision[i];
                find_final = true;
	    }
            else cout<<"There is no element in As_ob_info"<<endl;
            find_flag = true;
        }
    }
  
    int a = stringToNum<int>(formal_decision);


    int Formal_decision_=0;
	for (int i = 1; i <=get_length(a); i++)
	{
		Formal_decision_+=get_value(a,i)*power(2,i-1);
	}



    out<<"The formal decision is: "<<formal_decision<<" namely is: "<<Formal_decision_<<endl;
    cout<<"The formal decision is: "<<formal_decision<<" namely is: "<<Formal_decision_<<endl;

    if (Formal_decision_ < result_space.size())
    {
       formal_cost = result_space[Formal_decision_].total_cost_;
    }
    else formal_cost = -23333;

    if (find_final = true){
    formal_decision = final_decision;
    }
    else formal_decision = "22222";
    
    cout<<"The current decision is: "<<final_decision<<endl;
    cout<<"The minimum of the cost is: "<<min_cost<<endl;
    cout<<"The cost of the formal decision is: "<<formal_cost<<endl;
            if (find_final){
            out<<"Final decision is: "<<final_decision<<std::endl;
            out<<"The corresponding cost is: "<< min_cost<<endl;
            out<<"The cost of the formal decision is: "<<formal_cost<<endl;
	    if(!As_ob_info.empty()){
                youxiao = youxiao + 1;
                if(final_decision.compare(taibaoshou)!=0){
		xiangchao = xiangchao + 1;
		}
               }
            }

        if (!find_flag) {
            cout<<"f@ck!"<<endl;
            break;
        }
        cout <<"Path: ";
        for (int i = 0; i < horizon; i++)
        {
            cout << node.path_[i]<<" ";
        }
        cout << endl;
        out <<"Path: ";
        for (int i = 0; i < horizon; i++)
        {
            out << node.path_[i]<<" ";
        }
	

        for (int i = 0; i < horizon; i++)
        {
	    PATH1[i] = node.path_[i];
        }

        out << endl;
        cout <<"Total cost: "<<node.total_cost_ << endl;//Print total cost
        out <<"Total cost: "<<node.total_cost_ << endl;

        time_t end = clock();
        cout << "time cost: "<<double(end-start)/double(CLOCKS_PER_SEC) << endl;*/
        
         if (find_final){
	        cout<<"Final decision is: ";
	        for (size_t i=0;i<final_decision.size();i++)
			{
				cout<<final_decision[i]<<" ";
			}
		    cout<<endl;
	        cout<<"The corresponding cost is: "<< min_cost<<endl;
	            
	        
	        out<<"Final decision is: ";
	        for (size_t i=0;i<final_decision.size();i++)
			{
				out<<final_decision[i]<<" ";
			}
		    out<<endl;
	        out<<"The corresponding cost is: "<< min_cost<<endl;
	            /*out<<"The cost of the formal decision is: "<<formal_cost<<endl;*/ 
		    if(!As_ob_info.empty()){
                    int flag3 = 0;
	                youxiao = youxiao + 1;
                    for(size_t i = 0; i < final_decision.size();i++){
                        if(final_decision[i] == 1){
                            flag3 = flag3 + 1;
                        }
                    }
	                if(flag3 != 0){
						xiangchao = xiangchao + 1;
					}
	        }
        }

        if (!find_flag) {
            cout<<"f@ck!"<<endl;
            break;
        }
        cout <<"Path: ";
        for (int i = 0; i < horizon; i++)
        {
            cout << node.path_[i]<<" ";
        }
        cout << endl;
        out <<"Path: ";
        for (int i = 0; i < horizon; i++)
        {
            out << node.path_[i]<<" ";
        }
	

        for (int i = 0; i < horizon; i++)
        {
	    PATH1[i] = node.path_[i];
        }

        out << endl;
        cout <<"Total cost: "<<node.total_cost_ << endl;//Print total cost
        out <<"Total cost: "<<node.total_cost_ << endl;

        time_t end = clock();
        cout << "time cost: "<<double(end-start)/double(CLOCKS_PER_SEC) << endl;



        quadprogpp::Matrix<double> A;
        quadprogpp::Vector<double> b;
        A.resize(0,30,2);
        b.resize(0,2);
        if (!As_ob_info.empty()){
            //cout << "find decision: "<< final_decision << endl;
            A.resize(0,30,As_ob_info.size());
            b.resize(0,As_ob_info.size());
            for(size_t j = 0; j < As_ob_info.size(); j++){
                if (final_decision[j] == 1) {
                    alglib::spline1ddiff(spl_ref_xs, mycar->s_now + As_ob_info[j].s_b+5, xx, dx, d2x);
                    alglib::spline1ddiff(spl_ref_ys, mycar->s_now + As_ob_info[j].s_b+5, yy, dy, d2y);
                    cout<<"s_b x "<<xx<<"s_b y "<<yy<<endl;
                    if (As_ob_info[j].t_i < 1.0/3){
                        if (dx>0) {
                            A.s(-3*As_ob_info[j].t_i, 2 * int(As_ob_info[j].t_i * 3), j);
                            A.s(-dy/dx*3*As_ob_info[j].t_i, 2 * int(As_ob_info[j].t_i * 3) + 1, j);
                            b.s(-dy/dx*yy-xx+(1-3*As_ob_info[j].t_i)*mycar_state_now.x_+dy/dx*(1-3*As_ob_info[j].t_i)*mycar_state_now.y_,j);
                        } else if (dx<0){
                            A.s(3*As_ob_info[j].t_i, 2 * int(As_ob_info[j].t_i * 3) - 2, j);
                            A.s(dy/dx*3*As_ob_info[j].t_i, 2 * int(As_ob_info[j].t_i * 3) - 1, j);
                            b.s(dy/dx*yy+xx-(1-3*As_ob_info[j].t_i)*mycar_state_now.x_-dy/dx*(1-3*As_ob_info[j].t_i)*mycar_state_now.y_,j);
                        } else if (dy>0){
                            A.s(-1*3*As_ob_info[j].t_i,2 * int(As_ob_info[j].t_i * 3) - 2, j);
                            b.s(-yy+(1-3*As_ob_info[j].t_i)*mycar_state_now.y_,j);
                        } else if (dy<0){
                            A.s(1*3*As_ob_info[j].t_i,2 * int(As_ob_info[j].t_i * 3) - 1, j);
                            b.s(yy-(1-3*As_ob_info[j].t_i)*mycar_state_now.y_,j);
                        }
                    }else{
                        if (dx>0) {
                            A.s(-1, 2 * int(As_ob_info[j].t_i * 3) - 2, j);
                            A.s(-dy/dx, 2 * int(As_ob_info[j].t_i * 3) - 1, j);
                            b.s(-dy/dx*yy-xx,j);
                        } else if (dx<0){
                            A.s(1, 2 * int(As_ob_info[j].t_i * 3) - 2, j);
                            A.s(dy/dx, 2 * int(As_ob_info[j].t_i * 3) - 1, j);
                            b.s(dy/dx*yy+xx,j);
                        } else if (dy>0){
                            A.s(-1,2 * int(As_ob_info[j].t_i * 3) - 2, j);
                            b.s(-yy,j);
                        } else if (dy<0){
                            A.s(1,2 * int(As_ob_info[j].t_i * 3) - 1, j);
                            b.s(yy,j);
                        }
                    }

                }
                if (final_decision[j] == 0) {
                    alglib::spline1ddiff(spl_ref_xs, mycar->s_now + As_ob_info[j].s_f-5, xx, dx, d2x);
                    alglib::spline1ddiff(spl_ref_ys, mycar->s_now + As_ob_info[j].s_f-5, yy, dy, d2y);
                    cout<<"s_f x "<<xx<<"  "<<dx<<"  s_f y "<<yy<<" "<<dy<<endl;
                    if (As_ob_info[j].t_o < 1.0/3){
                        if (dx>0) {
                            A.s(3*As_ob_info[j].t_o, 2 * int(As_ob_info[j].t_o * 3+1), j);
                            A.s(dy/dx*3*As_ob_info[j].t_o, 2 * int(As_ob_info[j].t_o * 3+1) + 1, j);
                            b.s(dy/dx*yy+xx-(1-3*As_ob_info[j].t_o)*mycar_state_now.x_-dy/dx*(1-3*As_ob_info[j].t_o)*mycar_state_now.y_,j);
                        } else if (dx<0){
                            A.s(-1*3*As_ob_info[j].t_o, 2 * int(As_ob_info[j].t_o * 3+1) - 2, j);
                            A.s(-dy/dx*3*As_ob_info[j].t_o, 2 * int(As_ob_info[j].t_o * 3+1) - 1, j);
                            b.s(-dy/dx*yy-xx+(1-3*As_ob_info[j].t_o)*mycar_state_now.x_+dy/dx*(1-3*As_ob_info[j].t_o)*mycar_state_now.y_,j);
                        } else if (dy>0){
                            A.s(1*3*As_ob_info[j].t_o,2 * int(As_ob_info[j].t_o * 3) - 2, j);
                            b.s(yy-(1-3*As_ob_info[j].t_o)*mycar_state_now.y_,j);
                        } else if (dy<0){
                            A.s(-1*3*As_ob_info[j].t_o,2 * int(As_ob_info[j].t_o * 3) - 1, j);
                            b.s(-yy+(1-3*As_ob_info[j].t_o)*mycar_state_now.y_,j);
                        }
                    }else{
                        if (dx>0) {
                            A.s(1, 2 * int(As_ob_info[j].t_o * 3) , j);
                            A.s(dy/dx, 2 * int(As_ob_info[j].t_o * 3) + 1, j);
                            b.s(dy/dx*yy+xx,j);
                        } else if (dx<0){
                            A.s(-1, 2 * int(As_ob_info[j].t_o * 3) , j);
                            A.s(-dy/dx, 2 * int(As_ob_info[j].t_o * 3) + 1, j);
                            b.s(-dy/dx*yy-xx,j);
                        } else if (dy>0){
                            A.s(1,2 * int(As_ob_info[j].t_o * 3) , j);
                            b.s(yy,j);
                        } else if (dy<0){
                            A.s(-1,2 * int(As_ob_info[j].t_o * 3) + 1, j);
                            b.s(-yy,j);
                        }
                    }

                }
            }
        }
        
        vector <Node>().swap(result_space);
        vector <As_choice>().swap(As_ob_info);

        double v1_planed[5];                      // to calculate the output of Astar
        //double DT = 1.0;
        double s1_planed[5];
        double x1_planed[5];
        double y1_planed[5];
        double xx1_planed[5];
        double yy1_planed[5];
        double VS = V_s;


        cout<<"V from As: "<<V_s;
        if  (V_s + node.path_[0]*DT < 0){
            s1_planed[0] = abs(V_s*V_s/(2*node.path_[0]));
            V_s = 0;
            v1_planed[0] = V_s;
        } else{
            s1_planed[0] = V_s*DT + 0.5*node.path_[0]*DT*DT;
            V_s = V_s + node.path_[0]*DT;
            v1_planed[0] = V_s;
        }
        cout<<" "<<V_s;
        if  (V_s + node.path_[1]*DT < 0){
            s1_planed[1] = s1_planed[0] +abs(V_s*V_s/(2*node.path_[1]));
            V_s = 0;
	    v1_planed[1] = V_s;
        } else{
            s1_planed[1] = s1_planed[0]+ V_s*DT + 0.5*node.path_[1]*DT*DT;
            V_s = V_s + node.path_[1]*DT;
	    v1_planed[1] = V_s;
        }
        cout<<" "<<V_s;

        if  (V_s + node.path_[2]*DT < 0){
            s1_planed[2] = s1_planed[1] + abs(V_s*V_s/(2*node.path_[2]));
            V_s = 0;
	    v1_planed[2] = V_s;
        } else{
            s1_planed[2] = s1_planed[1]+ V_s*DT + 0.5*node.path_[2]*DT*DT;
            V_s = V_s + node.path_[2]*DT;
	    v1_planed[2] = V_s;
        }
        cout<<" "<<V_s;
        
        if  (V_s + node.path_[3]*DT < 0){
            s1_planed[3] = s1_planed[2] + abs(V_s*V_s/(2*node.path_[3]));
            V_s = 0;
	    v1_planed[3] = V_s;
        } else{
            s1_planed[3] = s1_planed[2]+ V_s*DT + 0.5*node.path_[3]*DT*DT;
            V_s = V_s + node.path_[3]*DT;
	    v1_planed[3] = V_s;
        }
        cout<<" "<<V_s;
        
        if  (V_s + node.path_[4]*DT < 0){
            s1_planed[4] = s1_planed[3] + abs(V_s*V_s/(2*node.path_[4]));
            V_s = 0;
	    v1_planed[4] = V_s;
        } else{
            s1_planed[4] = s1_planed[3]+ V_s*DT + 0.5*node.path_[4]*DT*DT;
            V_s = V_s + node.path_[4]*DT;
	    v1_planed[4] = V_s;
        }
        cout<<" "<<V_s<<endl;

        for (int i = 0; i < 5; i++){
            alglib::spline1ddiff(spl_ref_xs, s1_planed[i]+ mycar->s_now, xx, dx, d2x);
            alglib::spline1ddiff(spl_ref_ys, s1_planed[i]+ mycar->s_now, yy, dy, d2y);
            x1_planed[i]=xx;
            y1_planed[i]=yy;
            xx1_planed[i] = dx;
            yy1_planed[i] = dy;
        }

        double traj1_[10];

        for(int i = 0; i < 5; i++){
            traj1_[2*i]=x1_planed[i];
            traj1_[2*i+1]=y1_planed[i];
        }



        double s_planed[15];                            //to have more points(from 5 to 15)
        double x_planed[15];
        double y_planed[15];
        double xx_planed[15];
        double yy_planed[15];
        double dt=DT/3.0;
        V_s = VS;
        double ss;

        if  (V_s + node.path_[0]*DDTT < 0){            //to calculate the ouput of Astar when time is 0.1s
            //s_planed[0] = V_s*V_s/(-2*node.path_[0]);
            ss = mycar_state_now.x_ + V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            //s_planed[0] = V_s*dt + 0.5*node.path_[0]*dt*dt;
            ss = mycar_state_now.x_ + V_s*DDTT + 0.5*node.path_[0]*DDTT*DDTT; 
            V_s = V_s + node.path_[0]*DDTT;
        }

        V_s = VS;

        cout<<"V from As: "<<V_s;
        out<<"V from As: "<<V_s;
        int j = 3;
        if  (V_s + node.path_[0]*dt < 0){
            s_planed[0] = V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            s_planed[0] = V_s*dt + 0.5*node.path_[0]*dt*dt;
            V_s = V_s + node.path_[0]*dt;
        }
        cout<<" "<<V_s;
        out<<" "<<V_s;
        if  (V_s + node.path_[0]*dt < 0){
            s_planed[1] = s_planed[0] + V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            s_planed[1] = s_planed[0]+ V_s*dt + 0.5*node.path_[0]*dt*dt;
            V_s = V_s + node.path_[0]*dt;
        }
        cout<<" "<<V_s;
        out<<" "<<V_s;

        if  (V_s + node.path_[0]*dt < 0){
            s_planed[2] = s_planed[1] + V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            s_planed[2] = s_planed[1]+ V_s*dt + 0.5*node.path_[0]*dt*dt;
            V_s = V_s + node.path_[0]*dt;
        }
        cout<<" "<<V_s;
        out<<" "<<V_s;

        for (int i = 1; i < 5; i++){

            if  (V_s + node.path_[i]*dt < 0){
                s_planed[j] = s_planed[j-1] + V_s*V_s/(-2*node.path_[i]);
                V_s = 0;
            } else{
                s_planed[j] = s_planed[j-1] + V_s*dt + 0.5*node.path_[i]*dt*dt;
                V_s = V_s + node.path_[i]*dt;
            }
            cout<<" "<<V_s;
            out<<" "<<V_s;

            if  (V_s + node.path_[i]*dt < 0){
                s_planed[j+1] = s_planed[j] + V_s*V_s/(-2*node.path_[i]);
                V_s = 0;
            } else{
                s_planed[j+1] = s_planed[j]+ V_s*dt + 0.5*node.path_[i]*dt*dt;
                V_s = V_s + node.path_[i]*dt;
            }
            cout<<" "<<V_s;
            out<<" "<<V_s;

            if  (V_s + node.path_[i]*dt < 0){
                s_planed[j+2] = s_planed[j+1] + V_s*V_s/(-2*node.path_[i]);
                V_s = 0;
            } else{
                s_planed[j+2] = s_planed[j+1]+ V_s*dt + 0.5*node.path_[i]*dt*dt;
                V_s = V_s + node.path_[i]*dt;
            }
            cout<<" "<<V_s;
            out<<" "<<V_s;

            j=j+3;
        }
        out<<endl;

        for (int i = 0; i < 15; i++){
            alglib::spline1ddiff(spl_ref_xs, s_planed[i]+ mycar->s_now, xx, dx, d2x);
            alglib::spline1ddiff(spl_ref_ys, s_planed[i]+ mycar->s_now, yy, dy, d2y);
            x_planed[i]=xx;
            y_planed[i]=yy;
            xx_planed[i] = dx;
            yy_planed[i] = dy;
        }
        cout<<endl;

        alglib::spline1ddiff(spl_ref_xs, ss+ mycar->s_now, xx, dx, d2x);
        alglib::spline1ddiff(spl_ref_ys, ss+ mycar->s_now, yy, dy, d2y);


        AstarX.push_back(xx);
        AstarY.push_back(yy);
        //AstarVX.push_back(xx_planed[2]);
        AstarVX.push_back(dx);
        AstarVY.push_back(dy);


        double traj_[30];

        for(int i = 0; i < 15; i++){
            traj_[2*i]=x_planed[i];
            traj_[2*i+1]=y_planed[i];
        }

        cout <<"S from Astar: ";
        cout <<0<<" ";
        for (int i = 0; i < 15; i++)
        {
            cout << s_planed[i]<<" ";
        }
        cout << endl;

        out <<"S from Astar: ";
        out <<0<<" ";
        for (int i = 0; i < 15; i++)
        {
            out << s_planed[i]<<" ";
        }
        out << endl;

        cout <<"X from Astar: ";
        cout <<mycar_state_now.x_<<" ";
        for (int i = 0; i < 15; i++)
        {
            cout << x_planed[i]<<" ";
        }
        cout << endl;

        out <<"X from Astar: ";
        out <<mycar_state_now.x_<<" ";
        for (int i = 0; i < 15; i++)
        {
            out << x_planed[i]<<" ";
        }
        out << endl;

        cout <<"Y from Astar: ";
        cout <<mycar_state_now.y_<<" ";
        for (int i = 0; i < 15; i++)
        {
            cout << y_planed[i]<<" ";
        }
        cout << endl;

        out <<"Y from Astar: ";
        out <<mycar_state_now.y_<<" ";
        for (int i = 0; i < 15; i++)
        {
            out << y_planed[i]<<" ";
        }
        out << endl;

        quadprogpp::Matrix<double>  Aeq;
        quadprogpp::Vector<double>  beq, Xx;
        /*Q.resize(0,KT,KT);
        Qv.resize(0,Tp,Tp);
        Av.resize(0,Tp,Tp);
        p.resize(0,KT);
        pv.resize(0,Tp);
        bv.resize(0,Tp);
        qopt2.resize(0,KT);
        qopt3.resize(0,KT);*/
        Xx.resize(0,30);
        Aeq.resize(0,30,4);
        beq.resize(0,4);
        /*phi4.resize(0,Tp);*/
        double xinit,yinit,vxinit,vyinit,axinit,ayinit;

        //xinit=x1+32;yinit=lanecenter1;
        //vxinit=7;vyinit=0;
        //axinit=0;ayinit=0;

        //alglib::spline1ddiff(spl_ref_xs, mycar->s_now, xx, dx, d2x);
        //alglib::spline1ddiff(spl_ref_ys, mycar->s_now, yy, dy, d2y);


        xinit=mycar_state_now.x_;
        yinit=mycar_state_now.y_;
        vxinit=mycar_state_now.vx_;
        vyinit=mycar_state_now.vy_;
        cout<<"vx: "<< mycar_state_now.vx_ <<"  vy: "<<mycar_state_now.vy_<<endl;

        /*if (time_now >= DTT){
            //axinit=(1000.0/DTT)*(mycar_state_now.vx_ - mycar->getstate(time_now-DTT).vx_);
            //ayinit=(1000.0/DTT)*(mycar_state_now.vy_ - mycar->getstate(time_now-DTT).vy_);
            double a = (1000.0/DTT)*(sqrt(mycar_state_now.vx_*mycar_state_now.vx_ + mycar_state_now.vy_*mycar_state_now.vy_)-sqrt( mycar->getstate(time_now-DTT).vx_* mycar->getstate(time_now-DTT).vx_ +  mycar->getstate(time_now-DTT).vy_* mycar->getstate(time_now-DTT).vy_));
            axinit = a*cos(mycar_state_now.psi_rad_);
            ayinit = a*sin(mycar_state_now.psi_rad_);
            //cout<<"  ax: "<<axinit<<"  ay: "<<ayinit<<endl;
            cout<<"a: "<< a <<"  ax: "<<axinit<<"  ay: "<<ayinit<<endl;
        } else{

            axinit=0;
            ayinit=0;
        }*/

        double init[6];
        double wacc,wjerk,wdist;
        double weight[3];
        int LENGTH = 5;
        double traj2_[30];

        for (int zzz = 0; zzz < 2; zzz++){            //to decide which initializaion to be used for QP
            if (zzz == 0){
                axinit=0;
                ayinit=0;
            }
            else{
                axinit=  D2X.back();
                ayinit=  D2Y.back();
            }
            //axinit=0;
            //ayinit=0;
            //axinit=D2X.back();
            //ayinit=D2Y.back();
            //double init[6];
            init[0]=xinit;
            init[1]=yinit;
            init[2]=vxinit;
            init[3]=vyinit;
            init[4]=axinit;
            init[5]=ayinit;
            //double wacc,wjerk,wdist;
            wacc=0.5;wjerk=4;wdist=40;
            //wvacc=1;wvjerk=0.1;wvdist=100;
            //double weight[3];
            weight[0]=wacc;
            weight[1]=wjerk;
            weight[2]=wdist;

            //int LENGTH = 5;
            //double DT = 1.0;
            smoother sm1;
            sm1.setting(LENGTH,init,weight,traj1_,DT);
            sm1.setupAb(Aeq,A,beq,b);
            sm1.smoothing(Xx);

            

            /*weight[3]=wvacc;
            weight[4]=wvjerk;
            weight[5]=wvdist;*/
            //QPSetup(init,weight,dt,Tp,K,Q,Qv,Av,p,pv,bv);


            /*for (int i = 0; i < 5 ; i++) {

                qopt3.s(0,6*i);
                qopt3.s(0,6*i+1);
                qopt3.s(0,6*i+2);
                qopt3.s(0,6*i+3);
                qopt3.s(traj_[6*i+4],6*i+4);
                qopt3.s(traj_[6*i+5],6*i+5);
            }

            p -= 2*wdist*qopt3;*/

            //solve_quadprog(Q,p,Aeq,-beq,-A,b,qopt4);

            //double traj2_[30];

            for(int i = 0; i < 15; i++){
                traj2_[2*i] = Xx[2*i];
                traj2_[2*i+1] = Xx[2*i+1];
            }
            
            
            
            
            //traj_[i]=Xx[i];

            cout<<zzz<<endl;
            out<<zzz<<endl;

            cout <<"X from QP: ";
            cout <<mycar_state_now.x_<<" ";
            for (int i = 0; i < 15; i++)
            {
                cout <<traj2_[2*i]<<" ";
            }
            cout << endl;

            out <<"X from QP: ";
            out <<mycar_state_now.x_<<" ";
            for (int i = 0; i < 15; i++)
            {
                out <<traj2_[2*i]<<" ";
            }
            out << endl;

            cout <<"Y from QP: ";
            cout <<mycar_state_now.y_<<" ";
            for (int i = 0; i < 15; i++)
            {
                cout << traj2_[2*i+1]<<" ";
            }
            cout << endl;

            out <<"Y from QP: ";
            out <<mycar_state_now.y_<<" ";
            for (int i = 0; i < 15; i++)
            {
                out << traj2_[2*i+1]<<" ";
            }
            out << endl;

         /// to get the speed by using alglib
            double t_tra[50];
            alglib::real_1d_array t_sub;
            alglib::real_1d_array t_theta;
            alglib::real_1d_array y_old;
            alglib::real_1d_array x_old;
            alglib::real_1d_array theta;
            y_old.setlength(16);
            x_old.setlength(16);
            t_sub.setlength(16);
            theta.setlength(50);
            t_theta.setlength(50);
            alglib::spline1dinterpolant spl_x;
            alglib::spline1dinterpolant spl_y;
            alglib::spline1dinterpolant spl_theta;

            double t_plus = 0;
            for (j = 0; j < 16; j++) {
                t_sub[j]= t_plus;
                t_plus = t_plus + 1.0/3.0;
            }

            for(int i = 0; i < 50; i++) {
                t_tra[i] = double(i)/10.0 ;
            }
            //alglib::spline1ddiff(spl_ref_xs, mycar->s_now, xx, dx, d2x);
            //alglib::spline1ddiff(spl_ref_ys, mycar->s_now, yy, dy, d2y);
            x_old[0] = mycar_state_now.x_;
            y_old[0] = mycar_state_now.y_;
            for (int j = 1; j < 16; j++) {
                x_old[j]= traj2_[2*(j-1)];
                y_old[j]= traj2_[2*j-1];
            }

            alglib::spline1dbuildcubic(t_sub, x_old, 16, 1, mycar_state_now.vx_, 2, 0.0, spl_x);
            alglib::spline1dbuildcubic(t_sub, y_old, 16, 1, mycar_state_now.vy_, 2, 0.0, spl_y);
            /*
            for (int i = 0; i < 50;i++){
                alglib::spline1ddiff(spl_x, t_tra[i], xx, dx, d2x);
                alglib::spline1ddiff(spl_y, t_tra[i], yy, dy, d2y);
                theta[i] = atan2(dy,dx);
                t_theta[i] = t_tra[i];
            }
            alglib::spline1dbuildcubic(t_theta, theta, spl_theta);

            double thth = 0;
            double dth = 0;
            double d2th = 0;

            alglib::spline1ddiff(spl_theta, 0.1, thth, dth, d2th);
            */

            double xxx_planed[15];
            double yyy_planed[15];

            int ii = 0;
            for (double kkk = 0.3333; kkk < 5.1; kkk = kkk + 0.3333){
                alglib::spline1ddiff(spl_x, kkk, xx, dx, d2x);
                alglib::spline1ddiff(spl_y, kkk, yy, dy, d2y);
                xxx_planed[ii] = dx;
                yyy_planed[ii] = dy;
                ii = ii + 1;
            }

            cout <<"VX from QP: ";
            cout <<mycar_state_now.vx_<<" ";
            for (int i = 0; i < 15; i++)
            {
                cout <<xxx_planed[i]<<" ";
            }
            cout << endl;

            out <<"VX from QP: ";
            out <<mycar_state_now.vx_<<" ";
            for (int i = 0; i < 15; i++)
            {
                out <<xxx_planed[i]<<" ";
            }
            out << endl;

            cout <<"VY from QP: ";
            cout <<mycar_state_now.vy_<<" ";
            for (int i = 0; i < 15; i++)
            {
                cout << yyy_planed[i]<<" ";
            }
            cout << endl;

            out <<"VY from QP: ";
            out <<mycar_state_now.vy_<<" ";
            for (int i = 0; i < 15; i++)
            {
                out << yyy_planed[i]<<" ";
            }
            out << endl;


            
            alglib::spline1ddiff(spl_x, 0.1, xx, dx, d2x);
            alglib::spline1ddiff(spl_y, 0.1, yy, dy, d2y);

            if(zzz == 0){
                //D2X.push_back(d2x);
                //D2Y.push_back(d2y);
                QPVX.push_back(dx);
                QPVY.push_back(dy);
                QPX.push_back(xx);
                QPY.push_back(yy);

            }

            if(zzz == 1){
                state new_state;
                new_state.time_ = time_now + DTT;
                new_state.length_ = 4.67;
                new_state.width_ = 1.74;
                new_state.psi_rad_ = atan2(dy,dx);
                if (abs(v1_planed[0]) < 0.2){
                    new_state.vx_ = 0;
                }
                else{
                    new_state.vx_ = dx;
                }
                //new_state.vx_ = dx;
                new_state.vy_ = dy;
                new_state.x_ = xx;
                new_state.y_ = yy;
                //if(zzz == 1){
                  //  D2X.push_back(d2x);
                   // D2Y.push_back(d2y);
                //}
                D2X.push_back(d2x);
                D2Y.push_back(d2y);
                //cout<<"The ax is: "<<d2x<<endl;


                cout<<"Now x by QP is: "<<new_state.x_<<endl;
                cout<<"Now y by QP is: "<<new_state.y_<<endl;

                out<<"Now x by QP is: "<<new_state.x_<<endl;
                out<<"Now y by QP is: "<<new_state.y_<<endl;

                cout<<"Now vx by QP is: "<<new_state.vx_<<endl;
                cout<<"Now vy by QP is: "<<new_state.vy_<<endl;

                out<<"Now vx by QP is: "<<new_state.vx_<<endl;
                out<<"Now vy by QP is: "<<new_state.vy_<<endl;

                mycar->car_state.push_back(new_state);
                //out.close();

                time_now = time_now + DTT;
                cout<<"Time_now is : "<<time_now<<endl;

                /*double vxvx[6];      
                double xxx[6];
                int numb = 0;
                for(double kk = 0.05; kk < 0.33; kk = kk + 0.05){
                    alglib::spline1ddiff(spl_x, kk, xx, dx, d2x);
                    alglib::spline1ddiff(spl_y, kk, yy, dy, d2y);
                    vxvx[numb] = dx;
                    xxx[numb] = xx;
                    numb = numb + 1;
                }
                out <<"vx interpolated: ";
                out <<VS<<" ";
                for (int i = 0; i < 6; i++)
                {
                    out <<vxvx[i]<<" ";
                }
                out << endl;

                out <<"x interpolated: ";
                out <<mycar_state_now.x_ <<" ";
                for (int i = 0; i < 6; i++)
                {
                    out <<xxx[i]<<" ";
                }
                out << endl;*/
                
            }

        }
        out.close();
      
    }

    time_t END = clock();
    cout << "The total time cost: "<<double(END-START)/double(CLOCKS_PER_SEC) << endl;
    //out << "The total time cost: "<<double(END-START)/double(CLOCKS_PER_SEC) << endl;

    char write_file_name1[100]="/home/mscsim/state_write_test/test_%s.txt";
    gettimeofday(&T_now,NULL);
    area=localtime(&(T_now.tv_sec));
    sprintf(write_file_name1,"../record/exc/exceltest_%s.csv",asctime(area));
    ofstream File_creat1(write_file_name1);
    File_creat1.close();

    std::ofstream out1;
    out1.open(write_file_name1,std::ios::out|ios::trunc);
    out1<<"track_id"<<","<<"timestamp"<<","<<"x by QP"<<","<<"y by QP"<<","<<"vx by QP"<<","<<"vy by QP"<<","<<"psi_rad"<<","<<"length"<<","<<"width"<<","<<"AstarX"<<","<<"AstarY"<<","<<"AstarVX"<<","<<"AstarVY"<<","<<"QPX"<<","<<"QPY"<<","<<"QPVX"<<","<<"QPVY"<<","<<"D2X"<<","<<"D2Y"<<endl;
    
    /*for(auto recorded_state : mycar->car_state){
        std::cout<<"0,"<< recorded_state.time_/1000 <<","<< recorded_state.time_ <<",car, x: "<< recorded_state.x_ << ", y: "<< recorded_state.y_<<", vx: "<<recorded_state.vx_<<", vy: "<<recorded_state.vy_<<", psi_rad: "<<recorded_state.psi_rad_<<","<<"4.67,1.74"<< std::endl;
        out1<<"0"<<","<<recorded_state.time_<<","<<recorded_state.x_<<","<<recorded_state.y_<<","<<recorded_state.vx_<<","<<recorded_state.vy_<<","<<recorded_state.psi_rad_<<","<<"4.67"<<","<<"1.74"<<endl;
 
    }*/

    for(size_t i = 0; i < mycar->car_state.size();i++){
        std::cout<<"0,"<< mycar->car_state[i].time_/1000 <<","<< mycar->car_state[i].time_ <<",car, x by QP: "<< mycar->car_state[i].x_ << ", y by QP: "<< mycar->car_state[i].y_<<", vx by QP: "<<mycar->car_state[i].vx_<<", vy by QP: "<<mycar->car_state[i].vy_<<", psi_rad: "<<mycar->car_state[i].psi_rad_<<","<<"4.67,1.74"<<","<<", AstarX: "<<AstarX[i]<<", AstarY: "<<AstarY[i]<<", AstarVX: "<<AstarVX[i]<<", AstarVY: "<<AstarVY[i]<<", QPX: "<<QPX[i]<<", QPY: "<<QPY[i]<<", QPVX: "<<QPVX[i]<<", QPVY: "<<QPVY[i]<<", D2X: "<<D2X[i]<<", D2Y: "<<D2Y[i]<<std::endl;
        out1<<"0"<<","<<mycar->car_state[i].time_<<","<<mycar->car_state[i].x_<<","<<mycar->car_state[i].y_<<","<<mycar->car_state[i].vx_<<","<<mycar->car_state[i].vy_<<","<<mycar->car_state[i].psi_rad_<<","<<"4.67"<<","<<"1.74"<<","<<AstarX[i]<<","<<AstarY[i]<<","<<AstarVX[i]<<","<<AstarVY[i]<<","<<QPX[i]<<","<<QPY[i]<<","<<QPVX[i]<<","<<QPVY[i]<<","<<D2X[i]<<","<<D2Y[i]<<endl;
 
    }

    cout<<"end time : "<<time_now<<" ms"<<endl;
    cout<<"youxiao:"<<youxiao<<endl;
    cout<<"zongshu:"<<zongshu<<endl;
    cout<<"xiangchao:"<<xiangchao<<endl;
    
}