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

int power(int a,int b)
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
float w_runover = 0;
float desiredV = 2;
double V_s;
std::vector<int> actions = {-4,-3,-2,-1,0,1,2,3}; //Define possible accelerations
double dt=0.5;
double dtt = 0.2;
double horizon = 5*dt;
double curvature[12500];
vector<double> PATH1(5,1);


 
std::mutex mutex_for_AS;
vector<Node> result_space;
std::vector<As_choice> As_ob_info;
 
 
 
 
 void As_cau(int i){
 	std::string binary = std::bitset<5>(i).to_string(); //to binary
    auto AstarSolver = SpeedProfAstarSolver(0.0, V_s, horizon, dt, actions,PATH1,i);//params: s0 (ego vehicle), v0 (ego vehicle), horizon, dt, actions
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
 }
 
 
 int main(){
    //double dt=0.5;
    string formal_decision = "00000";	
    char write_file_name[100]="/home/mscsim/state_write_test/test_%s.txt";
    timeval T_now;
    tm *area;
    gettimeofday(&T_now,NULL);
    area=localtime(&(T_now.tv_sec));
    sprintf(write_file_name,"../record/test_%s.txt",asctime(area));
    ofstream File_creat(write_file_name);
    File_creat.close();
 	int youxiao = 0;
 	int zongshu = 0;
 	int xiangchao = 0;
 	string taibaoshou = "00000";
 	
 	ifstream track_file("../record/new/15_1.5_0.1.csv", ios::in);
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
    inistate.length_ = 4.37; 
    inistate.width_ = 1.77;
    inistate.psi_rad_ = 0;
    inistate.vx_ = 1.5;
    inistate.vy_ = 0;
    inistate.x_ = -10;
    inistate.y_ = 0;
    mycar->car_state.push_back(inistate);
    
    int time_now = 0;
    
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
    double lengthpass = V_s*horizon+0.5*1*horizon*horizon;
    cout<<"the length of the longest possible path is: "<< lengthpass<<endl;
	
	std::vector<As_choice> follow_info;
	
	for (auto ob:data_car){
		double s_f;
        double s_b;
        double t_i;
        double t_o;
        double V_for_f;
        bool init_flag = false;
        
		auto ob_state = ob->getstate(time_now);
		
		for (int i = 0; i < (10*horizon + 1); i++){
			auto ob_state = ob->getstate(time_now + i*100);
			if(ob_state.time_ == time_now + i*100){
                                //cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                                //cout<<"ob_state.x_ is: "<<ob_state.x_<<"  ob_state.y_ is: "<<ob_state.y_<<endl;
                                //cout<<ob_state.y_ - (ob_state.length_)/2<<" "<<ob_state.y_ + (ob_state.length_)/2<<" "<<ob_state.x_ - (ob_state.width_)/2<<" "<<ob_state.x_ + (ob_state.width_)/2<<endl;
				bool ob_flag_f = false;
				bool ob_flag_b = false;
				for (int j = 0; j < (V_s*horizon+0.5*1*horizon*horizon)*100;j++){
				alglib::spline1ddiff(spl_ref_xs, mycar->s_now + 0.01*j, xx, dx, d2x);
                                alglib::spline1ddiff(spl_ref_ys, mycar->s_now + 0.01*j, yy, dy, d2y);    
                                //cout<<"xx is: "<<xx<<" yy is: "<<yy<<endl;
                    if ((yy + (ob_state.width_)/2> ob_state.y_ - (ob_state.length_)/2) and (yy - (ob_state.width_)/2< ob_state.y_ + (ob_state.length_)/2) and (xx + (ob_state.length_)/2> ob_state.x_ - (ob_state.width_)/2) and (xx - (ob_state.length_)/2< ob_state.x_ + (ob_state.width_)/2)){
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
				    	if (0.1*i > t_o) t_o = 0.1*i;
				    	ob_flag_b = true;
				    	break;
					}
			
			    }
			    if (ob_flag_f and !ob_flag_b) {
                                s_b = (V_s*horizon+0.5*1*horizon*horizon);
                                t_o = horizon;
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
	if(!As_ob_info.empty()) {
		for (int i = 0; i < pow(2,As_ob_info.size());i++){
			std::string binary = std::bitset<5>(i).to_string();
			run_over_decision.push_back(binary);
 	        auto AstarSolver = SpeedProfAstarSolver(0.0,V_s,horizon,dt,actions,PATH1,i);
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
		auto AstarSolver = SpeedProfAstarSolver(0.0, V_s, horizon, dt, actions,PATH1,0);//params: s0 (ego vehicle), v0 (ego vehicle), horizon, dt, actions
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
	}
	if (using_thread){
        for(int i = 0;i<AS_thread.size();i++){
            AS_thread[i]->join();
            delete AS_thread[i];
        }
    }
    
    double min_cost = INF;
    double formal_cost = -1;
    bool find_flag = false;
    bool find_final = false;
    Node node;
    string final_decision;
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
                final_decision = run_over_decision[i];
                find_final = true;
	    }
            else cout<<"There is no element in As_ob_info"<<endl;
            find_flag = true;
        }
    }

    for (int i = 0; i < result_space.size(); i++){
        cout<<"PATH: ";
        for (int j = 0; j < 5; j++){
            cout<<result_space[i].path_[j]<<" ";
        }
        cout<<endl;
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
        for (int i = 0; i < 5; i++)
        {
            cout << node.path_[i]<<" ";
        }
        cout << endl;
        out <<"Path: ";
        for (int i = 0; i < 5; i++)
        {
            out << node.path_[i]<<" ";
        }
	

        for (int i = 0; i < 5; i++)
        {
	    PATH1[i] = node.path_[i];
        }

        out << endl;
        cout <<"Total cost: "<<node.total_cost_ << endl;//Print total cost
        out <<"Total cost: "<<node.total_cost_ << endl;

        time_t end = clock();
        cout << "time cost: "<<double(end-start)/double(CLOCKS_PER_SEC) << endl;
        
        vector <Node>().swap(result_space);
        vector <As_choice>().swap(As_ob_info);


        double s_planed[15];
        double x_planed[15];
        double y_planed[15];
        double dd=dt/3.0;
        cout<<"V from As: "<<V_s;
        int j = 3;
        if  (V_s + node.path_[0]*dd < 0){
            s_planed[0] = V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            s_planed[0] = V_s*dd + 0.5*node.path_[0]*dd*dd;
            V_s = V_s + node.path_[0]*dd;
        }
        cout<<" "<<V_s;
        if  (V_s + node.path_[0]*dd < 0){
            s_planed[1] = s_planed[0] + V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            s_planed[1] = s_planed[0]+ V_s*dd + 0.5*node.path_[0]*dd*dd;
            V_s = V_s + node.path_[0]*dd;
        }
        cout<<" "<<V_s;

        if  (V_s + node.path_[0]*dd < 0){
            s_planed[2] = s_planed[1] + V_s*V_s/(-2*node.path_[0]);
            V_s = 0;
        } else{
            s_planed[2] = s_planed[1]+ V_s*dd + 0.5*node.path_[0]*dd*dd;
            V_s = V_s + node.path_[0]*dd;
        }
        cout<<" "<<V_s;

        for (int i = 1; i < 5; i++){

            if  (V_s + node.path_[i]*dd < 0){
                s_planed[j] = s_planed[j-1] + V_s*V_s/(-2*node.path_[i]);
                V_s = 0;
            } else{
                s_planed[j] = s_planed[j-1] + V_s*dd + 0.5*node.path_[i]*dd*dd;
                V_s = V_s + node.path_[i]*dd;
            }
            cout<<" "<<V_s;

            if  (V_s + node.path_[i]*dd < 0){
                s_planed[j+1] = s_planed[j] + V_s*V_s/(-2*node.path_[i]);
                V_s = 0;
            } else{
                s_planed[j+1] = s_planed[j]+ V_s*dd + 0.5*node.path_[i]*dd*dd;
                V_s = V_s + node.path_[i]*dd;
            }
            cout<<" "<<V_s;

            if  (V_s + node.path_[i]*dd < 0){
                s_planed[j+2] = s_planed[j+1] + V_s*V_s/(-2*node.path_[i]);
                V_s = 0;
            } else{
                s_planed[j+2] = s_planed[j+1]+ V_s*dd + 0.5*node.path_[i]*dd*dd;
                V_s = V_s + node.path_[i]*dd;
            }
            cout<<" "<<V_s;

            j=j+3;
        }
        for (int i = 0; i < 15; i++){
            alglib::spline1ddiff(spl_ref_xs, s_planed[i]+ mycar->s_now, xx, dx, d2x);
            alglib::spline1ddiff(spl_ref_ys, s_planed[i]+ mycar->s_now, yy, dy, d2y);
            x_planed[i]=xx;
            y_planed[i]=yy;
        }
        cout<<endl;

        /*double v_planed[5];
        //double dt=0.5;
        double s_planed[5];
        double x_planed[5];
        double y_planed[5];
        double xx_planed[5];
        double yy_planed[5];


        cout<<"V from As: "<<V_s;
        if  (V_s + node.path_[0]*dt < 0){
            s_planed[0] = abs(V_s*V_s/(2*node.path_[0]));
            V_s = 0;
            v_planed[0] = V_s;
        } else{
            s_planed[0] = V_s*dt + 0.5*node.path_[0]*dt*dt;
            V_s = V_s + node.path_[0]*dt;
            v_planed[0] = V_s;
        }
        cout<<" "<<V_s;
        if  (V_s + node.path_[1]*dt < 0){
            s_planed[1] = s_planed[0] +abs(V_s*V_s/(2*node.path_[1]));
            V_s = 0;
	    v_planed[1] = V_s;
        } else{
            s_planed[1] = s_planed[0]+ V_s*dt + 0.5*node.path_[1]*dt*dt;
            V_s = V_s + node.path_[1]*dt;
	    v_planed[1] = V_s;
        }
        cout<<" "<<V_s;

        if  (V_s + node.path_[2]*dt < 0){
            s_planed[2] = s_planed[1] + abs(V_s*V_s/(2*node.path_[2]));
            V_s = 0;
	    v_planed[2] = V_s;
        } else{
            s_planed[2] = s_planed[1]+ V_s*dt + 0.5*node.path_[2]*dt*dt;
            V_s = V_s + node.path_[2]*dt;
	    v_planed[2] = V_s;
        }
        cout<<" "<<V_s;
        
        if  (V_s + node.path_[3]*dt < 0){
            s_planed[3] = s_planed[2] + abs(V_s*V_s/(2*node.path_[3]));
            V_s = 0;
	    v_planed[3] = V_s;
        } else{
            s_planed[3] = s_planed[2]+ V_s*dt + 0.5*node.path_[3]*dt*dt;
            V_s = V_s + node.path_[3]*dt;
	    v_planed[3] = V_s;
        }
        cout<<" "<<V_s;
        
        if  (V_s + node.path_[4]*dt < 0){
            s_planed[4] = s_planed[3] + abs(V_s*V_s/(2*node.path_[4]));
            V_s = 0;
	    v_planed[4] = V_s;
        } else{
            s_planed[4] = s_planed[3]+ V_s*dt + 0.5*node.path_[4]*dt*dt;
            V_s = V_s + node.path_[4]*dt;
	    v_planed[4] = V_s;
        }
        cout<<" "<<V_s<<endl;;
        
        for (int i = 0; i < 5; i++){
            alglib::spline1ddiff(spl_ref_xs, s_planed[i]+ mycar->s_now, xx, dx, d2x);
            alglib::spline1ddiff(spl_ref_ys, s_planed[i]+ mycar->s_now, yy, dy, d2y);
            x_planed[i]=xx;
            y_planed[i]=yy;
            xx_planed[i] = dx;
            yy_planed[i] = dy;
        }*/

        double traj_[30];
        for(int i = 0; i < 15; i++){
            traj_[2*i] = x_planed[i];
            traj_[2*i+1] = y_planed[i];
        }
        
        alglib::real_1d_array t_sub;
        alglib::real_1d_array y_old;
        alglib::real_1d_array x_old;
        
        y_old.setlength(16);
        x_old.setlength(16);
        t_sub.setlength(16);

        alglib::spline1dinterpolant spl_x;
        alglib::spline1dinterpolant spl_y;

        double t_plus = 0;
        for (int j = 0; j < 16; j++) {
            t_sub[j]= t_plus;
            t_plus = t_plus + dd;
        }

        x_old[0] = mycar_state_now.x_;
        y_old[0] = mycar_state_now.y_;

        for ( int j = 1; j < 16; j++) {
            x_old[j]= traj_[2*(j-1)];
            y_old[j]= traj_[2*j-1];
        }

        alglib::spline1dbuildcubic(t_sub, x_old, 16, 1, mycar_state_now.vx_, 2, 0.0, spl_x);
        alglib::spline1dbuildcubic(t_sub, y_old, 16, 1, mycar_state_now.vy_, 2, 0.0, spl_y);

        cout <<"S from Astar: ";
        cout <<mycar->s_now<<" ";
        for (int i = 0; i < 15; i++)
        {
            cout << mycar->s_now + s_planed[i]<<" ";
        }
        cout<<endl;
        
        cout <<"X from Astar: ";
        cout <<mycar_state_now.x_<<" ";
        for (int i = 0; i < 15; i++)
        {
            cout << x_planed[i]<<" ";
        }
        cout<<endl;
        
        cout <<"Y from Astar: ";
        cout <<mycar_state_now.y_<<" ";
        for (int i = 0; i < 15; i++)
        {
            cout << y_planed[i]<<" ";
        }
        cout<<endl;

        alglib::spline1ddiff(spl_x, dtt, xx, dx, d2x);
        alglib::spline1ddiff(spl_y, dtt, yy, dy, d2y);
        
        state new_state;
        new_state.time_ = time_now + dtt*1000;
        new_state.length_ = 4.67;
        new_state.width_ = 1.74;
        new_state.psi_rad_ = atan2(dy,dx);
        new_state.vx_ = dx;
        new_state.vy_ = dy;
        new_state.x_ = xx;
        new_state.y_ = yy;
        mycar->car_state.push_back(new_state);
	    out.close();
 	    
 	    time_now = time_now + dtt*1000;
            cout<<"Time_now is : "<<time_now<<endl;
 		
    }

    char write_file_name1[100]="/home/mscsim/state_write_test/test_%s.txt";
    gettimeofday(&T_now,NULL);
    area=localtime(&(T_now.tv_sec));
    sprintf(write_file_name1,"../record/excel/exceltest_%s.csv",asctime(area));
    ofstream File_creat1(write_file_name1);
    File_creat1.close();

    std::ofstream out1;
    out1.open(write_file_name1,std::ios::out|ios::trunc);
    out1<<"track_id"<<","<<"timestamp"<<","<<"x"<<","<<"y"<<","<<"vx"<<","<<"vy"<<","<<"psi_rad"<<","<<"length"<<","<<"width"<<endl;
    
    for(auto recorded_state : mycar->car_state){
        std::cout<<"0,"<< recorded_state.time_/1000 <<","<< recorded_state.time_ <<",car, x: "<< recorded_state.x_ << ", y: "<< recorded_state.y_<<", vx: "<<recorded_state.vx_<<", vy: "<<recorded_state.vy_<<", psi_rad: "<<recorded_state.psi_rad_<<","<<"4.67,1.74"<< std::endl;
                  out1<<"0"<<","<<recorded_state.time_<<","<<recorded_state.x_<<","<<recorded_state.y_<<","<<recorded_state.vx_<<","<<recorded_state.vy_<<","<<recorded_state.psi_rad_<<","<<"4.67"<<","<<"1.74"<<endl;
 
    }
    cout<<"end time : "<<time_now<<" ms"<<endl;
    cout<<"youxiao:"<<youxiao<<endl;
    cout<<"zongshu:"<<zongshu<<endl;
    cout<<"xiangchao:"<<xiangchao<<endl;
    
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
