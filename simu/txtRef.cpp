//
// Created by lcr on 7/26/19.
//

#include "txtRef.h"
#include <iostream>
#include <fstream>
#include <sstream>

template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

void txtRef::Creater_init(std::string ref_file){
    ref_file_ = ref_file;
    LineString2d reference(utils::getId());
    reference_ = reference;
    Ref_generate();
}

void  txtRef::Ref_generate(){
    alglib::real_1d_array y_ref;
    alglib::real_1d_array x_ref;
    alglib::real_1d_array s_ref;
    y_ref.setlength(81);
    x_ref.setlength(81);
    s_ref.setlength(81);

    std::ifstream track_file;
    track_file.open(ref_file_);
    std::string str;
    for (int j = 0; j < 80; ++j) {
        getline(track_file, str, ' ');
        x_ref[j]= stringToNum<double >(str);
    }
    getline(track_file, str, '\n');
    x_ref[80]= stringToNum<double >(str);
    for (int j = 0; j < 80; ++j) {
        getline(track_file, str, ' ');
        y_ref[j]= stringToNum<double >(str);
    }
    getline(track_file, str, '\n');
    y_ref[80]= stringToNum<double >(str);

    for (int k = 0; k < 81; ++k) {
        s_ref[k] = k;
    }


    alglib::spline1dbuildcubic(s_ref, x_ref, spl_ref_xs_);
    alglib::spline1dbuildcubic(s_ref, y_ref, spl_ref_ys_);
    y_ref.setlength(800000);
    x_ref.setlength(800000);
    s_ref.setlength(800000);
    s_ref[0]=0;
    for (int i = 0; i < 800000;i++){
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


alglib::spline1dinterpolant txtRef::getXS(){
    return spl_ref_xs_;
}
alglib::spline1dinterpolant txtRef::getYS(){

    return spl_ref_ys_;
}
LineString2d txtRef::getRef(){
    return reference_;
}