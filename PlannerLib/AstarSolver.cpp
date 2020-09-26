#include "AstarSolver.h"
#include <iostream>
#include "time.h"
#include <vector>
using namespace std;


static const float INF = 1000000;
static const float EPSLN = float(1e-3);
static const double pi = 3.1415926535;


/****************************    SpeedProfAstarSolver     ************************/

/// Initialization
/// \param s0
/// \param v0
/// \param horizon
/// \param dt
/// \param actions
SpeedProfAstarSolver::SpeedProfAstarSolver(const double s0, const double v0, const int horizon, const float dt, vector<int> actions, vector<double> PATH1) :
	s0_(s0),
	v0_(v0),
	horizon_(horizon),
	dt_(dt),
	actions_(actions),
	t_(boost::extents[300][250][16]),
    PATH1_(PATH1)
{
}

Node SpeedProfAstarSolver::plan()
{
    /*int cnt = 0;
         while (NUM_) {
             cnt++;
             NUM_ = NUM_ & (NUM_-1);
         }
    COST = cnt*w_runover_*cnt;*/
    vector<int > Passornot(5,2);
    start_node_ = Node(s0_,v0_,0,0,0,0,Passornot,0); //Node(const double s, const double v, const int layer, const float path_cost, const float heuristic_cost, const float total_cost) 

	cout<<"wrunover is: "<<this->w_runover_<<endl;

    unsigned int iteration = 0;
    t_[int((start_node_.s_ - s0_) * 2)][int(start_node_.v_*10)][start_node_.layer_] = Open;
    frontier_.Insert(start_node_);

    /*for(int i = 0; i < 5; i++){
        cout<<PATH1_[i]<<" ";
    }
    cout<<endl;*/


    /// The A* search
    while (true)
    {

        /// Pop the best node
        Node node = frontier_.GetMin();
        frontier_.DeleteMin();
        frontier_.Heapify(); 
        /// Check goal state
        if (SpeedProfAstarSolver::checkGoalState(node, 0))
        {
            return node;
        }
        //if (node.total_cost_ > INF) break;
        /// Add the node to the explored set
        t_[int((node.s_ - s0_) * 2)][int(node.v_*10)][node.layer_] = Explored;
        /// Expand nodes
        
        auto LAY = node.layer_;
        //cout<<"The path iteration currently is: "<<PATH1_[LAY]<<endl;
        for (auto child : expandLeafNode(node, actions_,PATH1_[LAY]))
        {
            if (t_[int((child.s_ - s0_) * 2)][int(child.v_*10)][child.layer_] != Open && t_[int((child.s_ - s0_) * 2)][int(child.v_*10)][child.layer_] != Explored)
            {
                frontier_.Insert(child);
                t_[int((child.s_ - s0_) * 2)][int(child.v_*10)][child.layer_] = Open;
                frontier_.Heapify(); 
            }
            else if (t_[int((child.s_ - s0_) * 2)][int(child.v_*10)][child.layer_] == Open)
            {
            	frontier_.Exchange(child,s0_);
                frontier_.Heapify();
            }
        }
    iteration = iteration+1;
    }
}

std::vector<Node> SpeedProfAstarSolver::expandLeafNode(const Node& node, vector<int> actions, const double path1)
{
	std::vector<Node> exp_leaf_list;
	int l = actions.size();
	auto s0 = node.s_;
	auto v0 = node.v_;
	auto dt = dt_;
	auto layer = node.layer_;
	
	for (int i = 0; i < l; i++)
	{
		auto passornot = node.Passornot_;
		double *COSTT = new double(node.COST_);
        int a = actions[i];
        auto s = s0 + v0*dt + 0.5*a*dt*dt;
        auto v = v0 + a*dt;
/*
        if ( desiredV_ - v0 < 1 && desiredV_ - v0 > 0 && actions[i] == 1){
            v = desiredV_;
            a = v - v0;
            s = s0 + v0*dt + 0.5*a*dt*dt;
        }


        if (v0 - desiredV_ < 2 && v0 - desiredV_ > 1 && actions[i] == -2){
            v = desiredV_;
            a = v - v0;
            s = s0 + v0*dt + 0.5*a*dt*dt;
        }
        if (v0 - desiredV_ < 1 && v0 - desiredV_ > 0 && actions[i] == -1) {
            v = desiredV_;
            a = v - v0;
            s = s0 + v0*dt + 0.5*a*dt*dt;
        }
*/

       
        if  (v0 + actions[i]*dt < 0){
            s = s0 + abs(v0*v0/(2*actions[i]));
            v = 0;
        }

		auto cost = getCost(node, a, path1,passornot,COSTT);
		//auto COSTT = getCOSTT(node,a,path1); 
        //auto Passornot = getPassornot(node, a, path1);
		auto path = node.path_;
		path.push_back(a);
		Node child;
		child.COST_ = (*COSTT);
		child.s_ = s;
		child.v_ = v;
		child.layer_ = node.layer_ + 1;
		if (child.layer_ == 5){
			//cout<<"The COSTT is: "<<COSTT<<endl;
			if ((*COSTT) == 0){
				(*COSTT) = -1;
			}
			cost = cost + (*COSTT)*9*this->w_runover_;
		}
		child.path_ = path;
		child.path_cost_ = node.path_cost_ + cost;
		child.heuristic_cost_ = 0;
		child.total_cost_ = child.path_cost_ + child.heuristic_cost_;
        child.Passornot_ = passornot;
        //std::cout<<"current node: "<<child.s_<<" "<<child.v_<<" "<<child.layer_<<" "<<child.total_cost_<<" "<<std::endl;
        exp_leaf_list.push_back(child);
		vector <int>().swap(passornot);
		delete COSTT;
		COSTT = NULL;
	}

	return exp_leaf_list;
}

/*double SpeedProfAstarSolver::getCost(const Node& node, double a, const double path1)
{
    double COST = 0; 
	int FLAG1 = 1;
	auto s0 = node.s_;
	auto v0 = node.v_;
	auto layer = node.layer_;
	auto path_cost = node.path_cost_;
	auto dt = dt_;
    bool to_zero_flag = false;
    auto passornot = node.Passornot_;
    //auto vfront = env_.Vehicles_[0];
    //auto vchange = env_.Vehicles_[1];
	//auto cross = env_.Crosses_[0];
    auto s = s0 + v0*dt + 0.5*a*dt*dt;
    auto v = v0 + a*dt;

    
    if  (v0 + a*dt < 0){
        s = s0 + abs(v0*v0/(2*a));
        v = 0;
        to_zero_flag = true;
    }

	double cur = this->curvature_[int((s - s0_) * 100)];
	if (cur < 0.002) cur = 0;
    auto a_n = cur*v*v;
	float margin = 5;
	if (v < 0) return INF;
	// must not enter obstacles
	for (auto vehi : env_.Vehicles_) {
        if (vehi.follow_){
            if (dt * (layer + 1) >= vehi.t0_) {
                if ( vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ - s -
                     margin < v*v/(2*2) - vehi.v0_*vehi.v0_/(2*2)) return INF;
                }
        }else if (dt * (layer + 1) >= vehi.t0_ and dt * (layer) <= vehi.t0_ ) {
            //if (s > vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ -
	        //        margin &&
	        //    s < vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ +
	        //        margin) {
            if (vehi.run_over_ ){
                if(v0 + a*dt >= 0){
                    if((s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer)) < vehi.s0_ + margin){
                        return INF;
                    }
                }
                else if(v0 + a*dt < 0){
                    if((s0 + abs(v0*v0/(2*a))) < vehi.s0_ + margin) {
                        return INF;
                    }
                }
            }
            else if (!vehi.run_over_){
                if(v0 + a*dt >= 0){
                    if((s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer)) >= vehi.s0_ - margin){
                        return INF;
                    }
                }
                else if(v0 + a*dt < 0){
                    if((s0 + abs(v0*v0/(2*a))) >= vehi.s0_ - margin) {
                        return INF;
                    }
                }
            }




	        //if  ((vehi.run_over_ ) and (s0 + v0*(vehi.t0_ - layer)+ abs(0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer)) < vehi.s0_ + margin)) return INF;
	        //else if ( (!vehi.run_over_) and (s0 + v0*(vehi.t0_ - layer)+ abs(0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer)) >= vehi.s0_ - margin)) return INF;

	    }
	}
	// must change the lane at end
	/*
	if (layer + 1 > horizon_ || (layer + 1 == horizon_&& s<vchange.s0_ + dt*(layer + 1)*vchange.v0_ + margin))
	{
		return INF;
	}

	// must not cross
	if (layer + 1>=cross.t_&&s>cross.s_)
	{
		return INF;
	}
    */
    
    
    
    /*double COSTT = this->w_a_ * a*a + this->w_v_ * (v - this->desiredV_)*(v - this->desiredV_)  + this->w_change_ * (a-path1)*(a-path1);
    if(isnan(COSTT)){
        return INF;
    }
    else{
        return COSTT;
    }
	//return this->w_a_ * a*a + this->w_v_ * (v - this->desiredV_)*(v - this->desiredV_) ;
    return this->w_a_ * a*a + this->w_v_ * (v - this->desiredV_)*(v - this->desiredV_) + this->w_an_ * a_n*a_n ;
}*/


double SpeedProfAstarSolver::getCost(const Node& node, double a, const double path1,vector<int> &passornot, double *COSTT)
{
	//double COST = 0;
	//auto COSTT = node.COST_; 
	int FLAG = 0;
	auto s0 = node.s_;
	auto v0 = node.v_;
	auto layer = node.layer_;
	auto path_cost = node.path_cost_;
	auto dt = dt_;
	bool to_zero_flag = false;
	//auto passornot = node.Passornot_;
    //auto vfront = env_.Vehicles_[0];
    //auto vchange = env_.Vehicles_[1];
	//auto cross = env_.Crosses_[0];
    auto s = s0 + v0*dt + 0.5*a*dt*dt;
    auto v = v0 + a*dt;

    
    if  (v0 + a*dt < 0){
        s = s0 + abs(v0*v0/(2*a));
        v = 0;
		to_zero_flag = true;
    }

	double cur = this->curvature_[int((s - s0_) * 100)];
	if (cur < 0.002) cur = 0;
    auto a_n = cur*v*v;
	float margin = 5;
	if (v < 0) FLAG = FLAG + 1;
	// must not enter obstacles
	/*for (size_t i = 0;i<env_.Vehicles_.size();i++){
		if (!env_.Vehicles_[i].follow_){
			cout<<"in cpp: s_f "<<env_.Vehicles_[i].s_f_<<" s_b "<<env_.Vehicles_[i].s_b_<<" v_0 "<<" t_o "<<env_.Vehicles_[i].t_o_<<" t_i "<<env_.Vehicles_[i].t_i_<<endl;
		}
	}*/
	for (size_t i = 0;i<env_.Vehicles_.size();i++){
		if (env_.Vehicles_[i].follow_){
            if (dt * (layer + 1) >= env_.Vehicles_[i].t_i_) {
                if ( env_.Vehicles_[i].s_f_ + (dt * (layer + 1) - env_.Vehicles_[i].t_i_) * env_.Vehicles_[i].v0_ - s -
                     margin < v*v/(2*2) - env_.Vehicles_[i].v0_*env_.Vehicles_[i].v0_/(2*2)) FLAG = FLAG + 1;
                }
        }else if(((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) or ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
        	if (((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) and (!((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ )))){
        		if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_i_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_i_ - layer)*(env_.Vehicles_[i].t_i_ - layer) >= env_.Vehicles_[i].s_b_ + margin)){
        			(*COSTT)++;
					//cout<<"03 ";
					passornot[i] = 3;
				}
				else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) >= env_.Vehicles_[i].s_b_ + margin)){
					(*COSTT)++;
					//cout<<"03 ";
					passornot[i] = 3;
				}
				else {
					passornot[i] = 0;
					//cout<<"13 "; 
				}
			}
			else if ((!((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_))) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
				/*if (passornot[i] == 1){

				}*/
				if (passornot[i] == 0){
					if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
						//cout<<"23 ";
					}
					else if ((to_zero_flag == true) and ((s0 + abs(v0*v0/(2*a))) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					}
					else {
						//cout<<"33 ";
						passornot[i] = 0;
					}

					/*if (passornot[i] == 0){
						//cout<<"43 ";
						return INF;
					}*/
					/*if (passornot[i] == 1){
						//cout<<"53 ";
						
					}*/
					/*if ((passornot[2*i] == 1) and (passornot[2*i+1] == 0)){
						//cout<<"63 ";
						
					}*/

				}

				if (passornot[i] == 0){
						//cout<<"43 ";
						FLAG = FLAG + 1;
				}
				/*if (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) >= env_.Vehicles_[i].s_f_ - margin){
					passornot[2*i+1] = 0;
					//cout<<"23 ";
				}
				else {
					//cout<<"33 ";
					passornot[2*i+1] = 1;
				}

				if ((passornot[2*i] == 0) and (passornot[2*i+1] == 0)){
					//cout<<"43 ";
					return INF;
				}
				if ((passornot[2*i] == 0) and (passornot[2*i+1] == 1)){
					//cout<<"53 ";
					
				}
				if ((passornot[2*i] == 1) and (passornot[2*i+1] == 0)){
					//cout<<"63 ";
					
				}*/
			}
			else if (((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
				if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_i_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_i_ - layer)*(env_.Vehicles_[i].t_i_ - layer) >= env_.Vehicles_[i].s_b_ + margin)){
					passornot[i] = 3;
				}
				else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) >= env_.Vehicles_[i].s_b_ + margin)){
					passornot[i] = 3;
				}
				else{
					if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					}
					else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					} 
					else{
						passornot[i] = 0;
					}
				}
				
				
				if (passornot[i] == 0){
					//cout<<"73 ";
					FLAG = FLAG + 1;
				}

				/*if ((flag1 ==0) and (flag2 == 1)){
					//cout<<"83 ";
					
				}*/

				if (passornot[i] == 3){
					(*COSTT)++;
					//cout<<"93 ";
					
				}
				
			}
		}
	}
	/*for (auto vehi : env_.Vehicles_) {
        if (vehi.follow_){
            if (dt * (layer + 1) >= vehi.t0_) {
                if ( vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ - s -
                     margin < v*v/(2*2) - vehi.v0_*vehi.v0_/(2*2)) return INF;
                }
        }else if ((dt * (layer + 1) >= vehi.t_i_ and dt * (layer) <= vehi.t_i_) or(dt * (layer + 1) >= vehi.t_o_ and dt * (layer) <= vehi.t_o_ )) {
            //if (s > vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ -
	        //        margin &&
	        //    s < vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ +
	        //        margin) {
	        //if  ((vehi.run_over_ ) and (s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer) < vehi.s0_ + margin)) return INF;
	        //else if ( (!vehi.run_over_) and (s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer) >= vehi.s0_ - margin)) return INF;
	        if ((s0 + v0*(vehi.t_i_ - layer)+ 0.5*a*(vehi.t_i_ - layer)*(vehi.t_i_ - layer) < vehi.s_b_ + margin) and (s0 + v0*(vehi.t_o_ - layer)+ 0.5*a*(vehi.t_o_ - layer)*(vehi.t_o_ - layer) >= vehi.s_f_ - margin)){
	        	return INF;
			}
			else if (s0 + v0*(vehi.t_i_ - layer)+ 0.5*a*(vehi.t_i_ - layer)*(vehi.t_i_ - layer) >= vehi.s_b_ + margin){ 
				COST = COST + this->w_runover_;
			}
			else if (s0 + v0*(vehi.t_o_ - layer)+ 0.5*a*(vehi.t_o_ - layer)*(vehi.t_o_ - layer) < vehi.s_f_ - margin){  
				COST = COST + 0;
			}

	    }
	}*/
	// must change the lane at end
	/*
	if (layer + 1 > horizon_ || (layer + 1 == horizon_&& s<vchange.s0_ + dt*(layer + 1)*vchange.v0_ + margin))
	{
		return INF;
	}

	// must not cross
	if (layer + 1>=cross.t_&&s>cross.s_)
	{
		return INF;
	}
    */
        /*cout<<COST;*/
		/*cout<<"83";*/
	if (FLAG != 0){
		return INF;
	}
	return this->w_a_ * a*a + this->w_v_ * (v - this->desiredV_)*(v - this->desiredV_) + this->w_an_ * a_n*a_n + this->w_change_ * (a-path1)*(a-path1);

}

/*double SpeedProfAstarSolver::getCOSTT(const Node& node, double a, const double path1)
{
	//double COST = 0;
	auto COSTT = node.COST_; 
	int FLAG1 = 1;
	auto s0 = node.s_;
	auto v0 = node.v_;
	auto layer = node.layer_;
	auto path_cost = node.path_cost_;
	auto dt = dt_;
	bool to_zero_flag = false;
	auto passornot = node.Passornot_;
    //auto vfront = env_.Vehicles_[0];
    //auto vchange = env_.Vehicles_[1];
	//auto cross = env_.Crosses_[0];
    auto s = s0 + v0*dt + 0.5*a*dt*dt;
    auto v = v0 + a*dt;

    
    if  (v0 + a*dt < 0){
        s = s0 + abs(v0*v0/(2*a));
        v = 0;
		to_zero_flag = true;
    }

	double cur = this->curvature_[int((s - s0_) * 100)];
	if (cur < 0.002) cur = 0;
    auto a_n = cur*v*v;
	float margin = 5;
	//if (v < 0) return INF;
	// must not enter obstacles
	/*for (size_t i = 0;i<env_.Vehicles_.size();i++){
		if (!env_.Vehicles_[i].follow_){
			cout<<"in cpp: s_f "<<env_.Vehicles_[i].s_f_<<" s_b "<<env_.Vehicles_[i].s_b_<<" v_0 "<<" t_o "<<env_.Vehicles_[i].t_o_<<" t_i "<<env_.Vehicles_[i].t_i_<<endl;
		}
	}
	for (size_t i = 0;i<env_.Vehicles_.size();i++){
		if (env_.Vehicles_[i].follow_){
            if (dt * (layer + 1) >= env_.Vehicles_[i].t_i_) {
                if ( env_.Vehicles_[i].s_f_ + (dt * (layer + 1) - env_.Vehicles_[i].t_i_) * env_.Vehicles_[i].v0_ - s -
                     margin < v*v/(2*2) - env_.Vehicles_[i].v0_*env_.Vehicles_[i].v0_/(2*2)) ;
                }
        }else if(((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) or ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
        	if (((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) and (!((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ )))){
        		if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_i_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_i_ - layer)*(env_.Vehicles_[i].t_i_ - layer) >= env_.Vehicles_[i].s_b_ + margin)){
        			COSTT = COSTT + 1;
					//cout<<"03 ";
					passornot[i] = 3;
				}
				else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) >= env_.Vehicles_[i].s_b_ + margin)){
					COSTT = COSTT + 1;
					//cout<<"03 ";
					passornot[i] = 3;
				}
				else {
					passornot[i] = 0;
					//cout<<"13 "; 
				}
			}*/
			//else if ((!((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_))) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
				/*if (passornot[i] == 1){

				}
				if (passornot[i] == 0){
					if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
						//cout<<"23 ";
					}
					else if ((to_zero_flag == true) and ((s0 + abs(v0*v0/(2*a))) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					}
					else {
						//cout<<"33 ";
						passornot[i] = 0;
					}*/

					/*if (passornot[i] == 0){
						//cout<<"43 ";
						return INF;
					}*/
					/*if (passornot[i] == 1){
						//cout<<"53 ";
						
					}*/
					/*if ((passornot[2*i] == 1) and (passornot[2*i+1] == 0)){
						//cout<<"63 ";
						
					}

				}

				/*if (passornot[i] == 0){
						//cout<<"43 ";
						return INF;
				}*/
				/*if (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) >= env_.Vehicles_[i].s_f_ - margin){
					passornot[2*i+1] = 0;
					//cout<<"23 ";
				}
				else {
					//cout<<"33 ";
					passornot[2*i+1] = 1;
				}

				if ((passornot[2*i] == 0) and (passornot[2*i+1] == 0)){
					//cout<<"43 ";
					return INF;
				}
				if ((passornot[2*i] == 0) and (passornot[2*i+1] == 1)){
					//cout<<"53 ";
					
				}
				if ((passornot[2*i] == 1) and (passornot[2*i+1] == 0)){
					//cout<<"63 ";
					
				}*//*
			}
			else if (((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
				if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_i_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_i_ - layer)*(env_.Vehicles_[i].t_i_ - layer) >= env_.Vehicles_[i].s_b_ + margin)){
					FLAG1 = 3;
				}
				else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) >= env_.Vehicles_[i].s_b_ + margin)){
					FLAG1 = 3;
				}
				else{
					if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) <= env_.Vehicles_[i].s_f_ - margin)){
						FLAG1 = 1;
					}
					else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) <= env_.Vehicles_[i].s_f_ - margin)){
						FLAG1 = 1;
					} 
					else{
						FLAG1 = 0;
					}
				}*/
				
				
				/*if (FLAG1 == 0){
					//cout<<"73 ";
					return INF;
				}*/

				/*if ((flag1 ==0) and (flag2 == 1)){
					//cout<<"83 ";
					
				}*/

				/*if (FLAG1 == 3){
					COSTT = COSTT + 1;
					//cout<<"93 ";
					
				}
				
			}
		}
	}*/
	/*for (auto vehi : env_.Vehicles_) {
        if (vehi.follow_){
            if (dt * (layer + 1) >= vehi.t0_) {
                if ( vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ - s -
                     margin < v*v/(2*2) - vehi.v0_*vehi.v0_/(2*2)) return INF;
                }
        }else if ((dt * (layer + 1) >= vehi.t_i_ and dt * (layer) <= vehi.t_i_) or(dt * (layer + 1) >= vehi.t_o_ and dt * (layer) <= vehi.t_o_ )) {
            //if (s > vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ -
	        //        margin &&
	        //    s < vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ +
	        //        margin) {
	        //if  ((vehi.run_over_ ) and (s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer) < vehi.s0_ + margin)) return INF;
	        //else if ( (!vehi.run_over_) and (s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer) >= vehi.s0_ - margin)) return INF;
	        if ((s0 + v0*(vehi.t_i_ - layer)+ 0.5*a*(vehi.t_i_ - layer)*(vehi.t_i_ - layer) < vehi.s_b_ + margin) and (s0 + v0*(vehi.t_o_ - layer)+ 0.5*a*(vehi.t_o_ - layer)*(vehi.t_o_ - layer) >= vehi.s_f_ - margin)){
	        	return INF;
			}
			else if (s0 + v0*(vehi.t_i_ - layer)+ 0.5*a*(vehi.t_i_ - layer)*(vehi.t_i_ - layer) >= vehi.s_b_ + margin){ 
				COST = COST + this->w_runover_;
			}
			else if (s0 + v0*(vehi.t_o_ - layer)+ 0.5*a*(vehi.t_o_ - layer)*(vehi.t_o_ - layer) < vehi.s_f_ - margin){  
				COST = COST + 0;
			}

	    }
	}*/
	// must change the lane at end
	/*
	if (layer + 1 > horizon_ || (layer + 1 == horizon_&& s<vchange.s0_ + dt*(layer + 1)*vchange.v0_ + margin))
	{
		return INF;
	}

	// must not cross
	if (layer + 1>=cross.t_&&s>cross.s_)
	{
		return INF;
	}
    */
        /*cout<<COST;*/
		/*cout<<"83";
	return COSTT;
	

}*/


/*vector<int > SpeedProfAstarSolver::getPassornot(const Node& node, double a, const double path1)
{
	double COST = 0; 
	int FLAG1 = 1;
	auto s0 = node.s_;
	auto v0 = node.v_;
	auto layer = node.layer_;
	auto path_cost = node.path_cost_;
	auto dt = dt_;
	bool to_zero_flag = false;
	auto passornot = node.Passornot_;
    //auto vfront = env_.Vehicles_[0];
    //auto vchange = env_.Vehicles_[1];
	//auto cross = env_.Crosses_[0];
    auto s = s0 + v0*dt + 0.5*a*dt*dt;
    auto v = v0 + a*dt;

    
    if  (v0 + a*dt < 0){
        s = s0 + abs(v0*v0/(2*a));
        v = 0;
		to_zero_flag = true;
    }

	double cur = this->curvature_[int((s - s0_) * 100)];
	if (cur < 0.002) cur = 0;
    auto a_n = cur*v*v;
	float margin = 5;
	//if (v < 0) return INF;
	// must not enter obstacles
	/*for (size_t i = 0;i<env_.Vehicles_.size();i++){
		if (!env_.Vehicles_[i].follow_){
			cout<<"in cpp: s_f "<<env_.Vehicles_[i].s_f_<<" s_b "<<env_.Vehicles_[i].s_b_<<" v_0 "<<" t_o "<<env_.Vehicles_[i].t_o_<<" t_i "<<env_.Vehicles_[i].t_i_<<endl;
		}
	}
	for (size_t i = 0;i<env_.Vehicles_.size();i++){
		if((!env_.Vehicles_[i].follow_) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) or ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
        	if (((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) and (!((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ )))){
        		if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_i_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_i_ - layer)*(env_.Vehicles_[i].t_i_ - layer) >= env_.Vehicles_[i].s_b_ + margin)){
        			COST = COST + 1;
					//cout<<"03 ";
					passornot[i] = 3;
				}
				else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) >= env_.Vehicles_[i].s_b_ + margin)){
					COST = COST + 1;
					//cout<<"03 ";
					passornot[i] = 3;
				}
				else {
					passornot[i] = 0;
					//cout<<"13 "; 
				}
			}
			else if ((!((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_))) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
				/*if (passornot[i] == 1){

				}
				if (passornot[i] == 0){
					if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
						//cout<<"23 ";
					}
					else if ((to_zero_flag == true) and ((s0 + abs(v0*v0/(2*a))) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					}
					else {
						//cout<<"33 ";
						passornot[i] = 0;
					}*/

					/*if (passornot[i] == 0){
						//cout<<"43 ";
						return INF;
					}*/
					/*if (passornot[i] == 1){
						//cout<<"53 ";
						
					}*/
					/*if ((passornot[2*i] == 1) and (passornot[2*i+1] == 0)){
						//cout<<"63 ";
						
					}

				}

				/*if (passornot[i] == 0){
						//cout<<"43 ";
						return INF;
				}*/
				/*if (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) >= env_.Vehicles_[i].s_f_ - margin){
					passornot[2*i+1] = 0;
					//cout<<"23 ";
				}
				else {
					//cout<<"33 ";
					passornot[2*i+1] = 1;
				}

				if ((passornot[2*i] == 0) and (passornot[2*i+1] == 0)){
					//cout<<"43 ";
					return INF;
				}
				if ((passornot[2*i] == 0) and (passornot[2*i+1] == 1)){
					//cout<<"53 ";
					
				}
				if ((passornot[2*i] == 1) and (passornot[2*i+1] == 0)){
					//cout<<"63 ";
					
				}*/
			/*}
			else if (((dt * (layer + 1) >= env_.Vehicles_[i].t_i_) and (dt * (layer) <= env_.Vehicles_[i].t_i_)) and ((dt * (layer + 1) >= env_.Vehicles_[i].t_o_) and (dt * (layer) <= env_.Vehicles_[i].t_o_ ))){
				if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_i_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_i_ - layer)*(env_.Vehicles_[i].t_i_ - layer) >= env_.Vehicles_[i].s_b_ + margin)){
					passornot[i] = 3;
				}
				else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) >= env_.Vehicles_[i].s_b_ + margin)){
					passornot[i] = 3;
				}
				else{
					if ((to_zero_flag == false) and (s0 + v0*(env_.Vehicles_[i].t_o_ - layer)+ 0.5*a*(env_.Vehicles_[i].t_o_ - layer)*(env_.Vehicles_[i].t_o_ - layer) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					}
					else if ((to_zero_flag == true) and (s0 + abs(v0*v0/(2*a)) <= env_.Vehicles_[i].s_f_ - margin)){
						passornot[i] = 1;
					} 
					else{
						passornot[i] = 0;
					}
				}
				
				
				/*if (FLAG1 == 0){
					//cout<<"73 ";
					return INF;
				}*/

				/*if ((flag1 ==0) and (flag2 == 1)){
					//cout<<"83 ";
					
				}*/

				/*if (FLAG1 == 3){
					COST = COST + 1;
					//cout<<"93 ";
					
				}
				
			}
		}
	}*/
	/*for (auto vehi : env_.Vehicles_) {
        if (vehi.follow_){
            if (dt * (layer + 1) >= vehi.t0_) {
                if ( vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ - s -
                     margin < v*v/(2*2) - vehi.v0_*vehi.v0_/(2*2)) return INF;
                }
        }else if ((dt * (layer + 1) >= vehi.t_i_ and dt * (layer) <= vehi.t_i_) or(dt * (layer + 1) >= vehi.t_o_ and dt * (layer) <= vehi.t_o_ )) {
            //if (s > vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ -
	        //        margin &&
	        //    s < vehi.s0_ + (dt * (layer + 1) - vehi.t0_) * vehi.v0_ +
	        //        margin) {
	        //if  ((vehi.run_over_ ) and (s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer) < vehi.s0_ + margin)) return INF;
	        //else if ( (!vehi.run_over_) and (s0 + v0*(vehi.t0_ - layer)+ 0.5*a*(vehi.t0_ - layer)*(vehi.t0_ - layer) >= vehi.s0_ - margin)) return INF;
	        if ((s0 + v0*(vehi.t_i_ - layer)+ 0.5*a*(vehi.t_i_ - layer)*(vehi.t_i_ - layer) < vehi.s_b_ + margin) and (s0 + v0*(vehi.t_o_ - layer)+ 0.5*a*(vehi.t_o_ - layer)*(vehi.t_o_ - layer) >= vehi.s_f_ - margin)){
	        	return INF;
			}
			else if (s0 + v0*(vehi.t_i_ - layer)+ 0.5*a*(vehi.t_i_ - layer)*(vehi.t_i_ - layer) >= vehi.s_b_ + margin){ 
				COST = COST + this->w_runover_;
			}
			else if (s0 + v0*(vehi.t_o_ - layer)+ 0.5*a*(vehi.t_o_ - layer)*(vehi.t_o_ - layer) < vehi.s_f_ - margin){  
				COST = COST + 0;
			}

	    }
	}*/
	// must change the lane at end
	/*
	if (layer + 1 > horizon_ || (layer + 1 == horizon_&& s<vchange.s0_ + dt*(layer + 1)*vchange.v0_ + margin))
	{
		return INF;
	}

	// must not cross
	if (layer + 1>=cross.t_&&s>cross.s_)
	{
		return INF;
	}
    */
        /*cout<<COST;*/
		/*cout<<"83";
	//return this->w_a_ * a*a + this->w_v_ * (v - this->desiredV_)*(v - this->desiredV_) + this->w_an_ * a_n*a_n + this->w_change_ * (a-path1)*(a-path1) + COST*this->w_runover_;
	return passornot;
}*/





bool SpeedProfAstarSolver::checkGoalState(const Node& node, int goalType)
{
	if (goalType == 0)
	{
		if (node.layer_ == horizon_)
		{
			return 1;
		}
		return 0;
	}
	else
	{
		return 0;
	}
}

