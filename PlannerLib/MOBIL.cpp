//
// Created by Chenran on 06/10/19.
//


#include "MOBIL.hpp"


/**********************

  -----          -----         -----
 |  n  |        |     |       |  s  |
  -----          -----         -----
 ===========================================   Lane line
                  ^
                  |
  -----        -----          -----
 |  o  |      |  c  |        |  q  |
  -----        -----          -----

 **********************/



MOBIL::MOBIL(double current_states[4], double front_states[4], double back_states[4], double next_front_states[4], double next_back_states[4]):
decision(false),
lng_pos(current_states[0]),velocity(current_states[1]),acceleration(current_states[2]),desire_v(current_states[3]),
q_lng_pos(front_states[0]),q_velocity(front_states[1]),q_acceleration(front_states[2]),q_desire_v(front_states[3]),
o_lng_pos(back_states[0]),o_velocity(back_states[1]),o_acceleration(back_states[2]),o_desire_v(back_states[3]),
s_lng_pos(next_front_states[0]),s_velocity(next_front_states[1]),s_acceleration(next_front_states[2]),s_desire_v(next_front_states[3]),
n_lng_pos(next_back_states[0]),n_velocity(next_back_states[1]),n_acceleration(next_back_states[2]),n_desire_v(next_back_states[3])
{
    if (acceleration < 0) decision = false;
    else if (n_lng_pos + MIN_DISTANCE > lng_pos || s_lng_pos - MIN_DISTANCE < lng_pos) decision = false;
    else{
        IDM c2s =  IDM(velocity,lng_pos,desire_v,s_lng_pos,s_velocity);
        IDM c2q = IDM(velocity,lng_pos,desire_v,q_lng_pos,q_velocity);
        IDM o2q = IDM(o_velocity,o_lng_pos,o_desire_v,q_lng_pos,q_velocity);
        IDM n2c = IDM(n_velocity,n_lng_pos,n_desire_v,lng_pos,velocity);
        double ac_tilde = c2s.acceleration;
        double ac = c2q.acceleration;
        double ao_tilde = o2q.acceleration;
        double an_tilde = n2c.acceleration;
        double an = n_acceleration;
        double ao = o_acceleration;
        if(ac_tilde - ac + POLITENESS_FACTOR * ( an_tilde - an + ao_tilde -ao) > CHANGEING_THRESHOLD)  decision = true;
    }

}
