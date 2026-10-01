#pragma once
#include <array>
#include <string>
#include <vector>
namespace trailer {
constexpr int Nodes=101, Bodies=4;
constexpr double Pi=3.14159265358979323846;
using Point=std::array<double,2>;
using Polygon=std::array<Point,4>;
using State=std::array<double,19>; // four [x,y,heading,speed], then acceleration, steering, steering rate
struct Scenario { int id; std::array<Point,4> start,goal; std::vector<Polygon> obstacles; std::vector<double> occupancy,potential; };
struct Trajectory { double duration=0, infeasibility=0; std::array<State,Nodes> states{}; };
struct Box { double xmin,xmax,ymin,ymax; };
using Corridors=std::array<std::array<Box,4>,Nodes>;
struct CollisionFlags { int node,body,obstacle,vertex; bool obstacle_vertex; };
Scenario load_scenario(const std::string&,int);
Trajectory initial_guess(const Scenario&);
Corridors build_corridors(const Scenario&,const Trajectory&);
std::vector<CollisionFlags> activate_constraints(const Scenario&,const Trajectory&);
Trajectory solve(const Scenario&,const Trajectory&,int,const Corridors&,const std::vector<CollisionFlags>&,const std::string&,const std::string&);
Polygon footprint(const State&,int);
bool inside(Point,const Polygon&);
double validate(const Scenario&,const Trajectory&);
void write_results(const Scenario&,const Trajectory&,const std::string&,double,double);
}
