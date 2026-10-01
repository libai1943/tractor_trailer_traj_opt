#include "planner.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv) {
  try {
    int id=1;std::string data="data/cases_data.bin",output="results",linear="ma97",hsl;
    if(const char* value=std::getenv("HSL_LIBRARY"))hsl=value;
    for(int i=1;i<argc;++i){std::string arg=argv[i];if(i+1>=argc)throw std::runtime_error("Options require a value.");std::string value=argv[++i];if(arg=="--case")id=std::stoi(value);else if(arg=="--data")data=value;else if(arg=="--output")output=value;else if(arg=="--linear-solver")linear=value;else if(arg=="--hsl-library")hsl=value;else throw std::runtime_error("Unknown option: "+arg);}
    using namespace trailer;using Clock=std::chrono::steady_clock;
    auto scene=load_scenario(data,id);auto start=Clock::now();double preprocessing=0;
    auto current=initial_guess(scene);Corridors boxes;
    for(int iteration=1;iteration<=5;++iteration){auto begin=Clock::now();boxes=build_corridors(scene,current);preprocessing+=std::chrono::duration<double>(Clock::now()-begin).count();current=solve(scene,current,2,boxes,{},linear,hsl);std::cout<<"LIOM "<<iteration<<" | T "<<current.duration<<" | residual "<<current.infeasibility<<std::endl;if(current.infeasibility<1e-3)break;}
    for(int iteration=1;iteration<=10;++iteration){auto begin=Clock::now();auto flags=activate_constraints(scene,current);preprocessing+=std::chrono::duration<double>(Clock::now()-begin).count();double old=current.duration;current=solve(scene,current,3,boxes,flags,linear,hsl);std::cout<<"TRMO "<<iteration<<" | T "<<current.duration<<" | improvement "<<old-current.duration<<std::endl;if(old-current.duration<1)break;}
    double planning=std::chrono::duration<double>(Clock::now()-start).count();double residual=validate(scene,current);
    write_results(scene,current,output,planning,preprocessing);
    std::cout<<"Validated | case "<<id<<" | planning "<<planning<<" s | preprocessing "<<preprocessing<<" s | dynamics residual "<<residual<<std::endl;
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
