#include "planner.hpp"
#include <casadi/casadi.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace trailer {
using casadi::SX;using casadi::DM;using casadi::Function;using casadi::DMDict;using casadi::Dict;using casadi::SXDict;
Trajectory solve(const Scenario& scene,const Trajectory& previous,int stage,const Corridors& boxes,
                 const std::vector<CollisionFlags>& flags,const std::string& linear_solver,const std::string& hsl) {
  const bool soft=stage==2;const int dimension=(soft?21:19)*Nodes+1;const double infinity=std::numeric_limits<double>::infinity();
  SX z=SX::sym("state",dimension),tf=z(dimension-1),dt=tf/(Nodes-1);
  auto at=[&](int i,int k)->SX{return z(k*Nodes+i);};
  auto square=[](SX v){return v*v;};
  std::vector<double> lower(dimension,-infinity),upper(dimension,infinity),initial(dimension,0),gl,gu;std::vector<SX> constraints;
  auto bound=[&](int i,int k,double lo,double hi){int ix=k*Nodes+i;lower[ix]=std::max(lower[ix],lo);upper[ix]=std::min(upper[ix],hi);};
  auto add=[&](SX v,double lo,double hi){constraints.push_back(v);gl.push_back(lo);gu.push_back(hi);};
  SX residual=0;
  auto motion=[&](SX v){if(soft)residual+=square(v);else add(v,0,0);};
  for(int i=0;i<Nodes;++i) {
    for(int k=0;k<19;++k)initial[k*Nodes+i]=previous.states[i][k];
    bound(i,3,-2.5,2.5);bound(i,16,-.25,.25);bound(i,17,-.7,.7);bound(i,18,-.5,.5);
    for(int b=0;b<4;++b){int k=4*b;
      if(soft){bound(i,k,-23.5858,23.5858);bound(i,k+1,-23.5858,23.5858);auto box=boxes[i][b];int xcol=b?k:19,ycol=b?k+1:20;
        bound(i,xcol,box.xmin,box.xmax);bound(i,ycol,box.ymin,box.ymax);
      }else {bound(i,k,previous.states[i][k]-3,previous.states[i][k]+3);bound(i,k+1,previous.states[i][k+1]-3,previous.states[i][k+1]+3);bound(i,k+2,previous.states[i][k+2]-Pi/4,previous.states[i][k+2]+Pi/4);}
      if(b){add(at(i,k+2)-at(i,k-2),-Pi/2+.1,Pi/2-.1);motion(at(i,k)-at(i,k-4)+3*cos(at(i,k+2)));motion(at(i,k+1)-at(i,k-3)+3*sin(at(i,k+2)));
        if(i>0||!soft)motion(at(i,k+3)-at(i,k-1)*cos(at(i,k-2)-at(i,k+2)));
        if(i>0)motion(3*(at(i,k+2)-at(i-1,k+2))-dt*at(i-1,k-1)*sin(at(i-1,k-2)-at(i-1,k+2)));
      }
    }
    if(soft){initial[19*Nodes+i]=previous.states[i][0]+.75*cos(previous.states[i][2]);initial[20*Nodes+i]=previous.states[i][1]+.75*sin(previous.states[i][2]);motion(at(i,0)+.75*cos(at(i,2))-at(i,19));motion(at(i,1)+.75*sin(at(i,2))-at(i,20));}
    if(i){motion(at(i,0)-at(i-1,0)-dt*at(i-1,3)*cos(at(i-1,2)));motion(at(i,1)-at(i-1,1)-dt*at(i-1,3)*sin(at(i-1,2)));motion(at(i,2)-at(i-1,2)-dt*at(i-1,3)*tan(at(i-1,17))/1.5);motion(at(i,3)-at(i-1,3)-dt*at(i-1,16));motion(at(i,17)-at(i-1,17)-dt*at(i-1,18));}
  }
  for(int i:{0,Nodes-1}){for(int b=0;b<4;++b)bound(i,4*b+2,0,0);for(int c:{3,16,17,18})bound(i,c,0,0);Point endpoint=i?scene.goal[0]:scene.start[0];bound(i,0,endpoint[0],endpoint[0]);bound(i,1,endpoint[1],endpoint[1]);}
  lower.back()=.1;upper.back()=soft?100:std::max(100.,std::ceil(previous.duration));initial.back()=previous.duration;
  using SymbolicPolygon=std::array<std::array<SX,2>,4>;
  std::array<std::array<SymbolicPolygon,4>,Nodes> corners;
  if(!soft){
    for(int i=0;i<Nodes;++i)for(int b=0;b<4;++b){SX x=at(i,4*b),y=at(i,4*b+1),t=at(i,4*b+2);double front=b?1:1.75,rear=b?1:.25;double dx[]={front,front,-rear,-rear},dy[]={1,-1,-1,1};
      for(int v=0;v<4;++v){corners[i][b][v]={x+dx[v]*cos(t)-dy[v]*sin(t),y+dx[v]*sin(t)+dy[v]*cos(t)};add(corners[i][b][v][0],-25,25);add(corners[i][b][v][1],-25,25);}
    }
    auto area_sum=[](const SymbolicPolygon& polygon,std::array<SX,2> point){SX sum=0;for(int j=0;j<4;++j){auto a=polygon[j],b=polygon[(j+1)%4];sum+=.5*fabs((a[0]-point[0])*(b[1]-point[1])-(a[1]-point[1])*(b[0]-point[0]));}return sum;};
    for(auto flag:flags){const auto& obstacle=scene.obstacles[flag.obstacle];const auto& body=corners[flag.node][flag.body];
      if(flag.obstacle_vertex)add(area_sum(body,{SX(obstacle[flag.vertex][0]),SX(obstacle[flag.vertex][1])}),4.1,infinity);
      else{SymbolicPolygon polygon;double twice_area=0;for(int j=0;j<4;++j){polygon[j]={SX(obstacle[j][0]),SX(obstacle[j][1])};twice_area+=obstacle[j][0]*obstacle[(j+1)%4][1]-obstacle[j][1]*obstacle[(j+1)%4][0];}double area=.5*std::abs(twice_area);add(area_sum(polygon,body[flag.vertex]),std::min(area+.1,area*1.01),infinity);}
    }
  }
  Dict options{{"print_time",false},{"ipopt.print_level",0},{"ipopt.tol",1e-3},{"ipopt.constr_viol_tol",1e-6},{"ipopt.max_iter",3000},{"ipopt.max_cpu_time",120.},{"ipopt.mu_strategy","adaptive"},{"ipopt.linear_solver",linear_solver}};
  if(!hsl.empty())options["ipopt.hsllib"]=hsl;
  SX cost=soft?tf+1e4*residual:tf;
  static Function cached_liom; static std::string cached_key;
  Function solver;
  if(soft&&!cached_liom.is_null()&&cached_key==linear_solver+hsl)solver=cached_liom;
  else {solver=casadi::nlpsol(soft?"liom":"trmo","ipopt",SXDict{{"x",z},{"f",cost},{"g",SX::vertcat(constraints)}},options);if(soft){cached_liom=solver;cached_key=linear_solver+hsl;}}
  auto answer=solver(DMDict{{"x0",DM(initial)},{"lbx",DM(lower)},{"ubx",DM(upper)},{"lbg",DM(gl)},{"ubg",DM(gu)}});
  if(!static_cast<bool>(solver.stats().at("success")))throw std::runtime_error("IPOPT: "+solver.stats().at("return_status").to_string());
  auto values=answer.at("x").get_elements();Trajectory result;result.duration=values.back();result.infeasibility=soft?(double(answer.at("f"))-result.duration)/1e4:0;
  for(int i=0;i<Nodes;++i)for(int k=0;k<19;++k)result.states[i][k]=values[k*Nodes+i];
  return result;
}
}
