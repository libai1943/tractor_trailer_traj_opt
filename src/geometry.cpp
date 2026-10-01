#include "planner.hpp"
#include "guiding_search.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace trailer {
bool inside(Point p,const Polygon& q) {bool a=true,b=true;for(int i=0;i<4;++i){auto u=q[i],v=q[(i+1)%4];double c=(v[0]-u[0])*(p[1]-u[1])-(v[1]-u[1])*(p[0]-u[0]);a=a&&c>=-1e-12;b=b&&c<=1e-12;}return a||b;}
Polygon footprint(const State& s,int body) {double x=s[4*body],y=s[4*body+1],t=s[4*body+2],front=body?1:1.75,rear=body?1:.25;Polygon q;double dx[]={front,front,-rear,-rear},dy[]={1,-1,-1,1};for(int k=0;k<4;++k)q[k]={x+dx[k]*cos(t)-dy[k]*sin(t),y+dx[k]*sin(t)+dy[k]*cos(t)};return q;}
Scenario load_scenario(const std::string& filename,int case_id) {
  if(case_id<1||case_id>2)throw std::runtime_error("case_id must be 1 or 2.");
  std::ifstream in(filename,std::ios::binary);if(!in)throw std::runtime_error("Cannot open cases_data.bin");
  auto read=[&](double* value,size_t n){in.read(reinterpret_cast<char*>(value),n*sizeof(double));if(!in)throw std::runtime_error("Incomplete case data.");};
  for(int id=1;id<=2;++id) {Scenario s;s.id=id;double number;read(&number,1);s.obstacles.resize(int(number));
    for(auto& p:s.start)read(p.data(),2);for(auto& p:s.goal)read(p.data(),2);
    for(auto& polygon:s.obstacles)for(auto& p:polygon)read(p.data(),2);
    s.occupancy.resize(10201);s.potential.resize(10201);read(s.occupancy.data(),10201);read(s.potential.data(),10201);
    if(id==case_id)return s;
  }throw std::runtime_error("Invalid case data.");
}
Trajectory initial_guess(const Scenario& scenario) {
  Trajectory r;std::array<double,4> times{};
  for(int body=0;body<4;++body) {
    Point first=scenario.start[body],last=scenario.goal[body];if(body==0){first[0]+=.75;last[0]+=.75;}
    std::array<double,2*Nodes> path{};
    if(search_guiding_path(scenario.occupancy.data(),scenario.potential.data(),first.data(),last.data(),Nodes,path.data()))throw std::runtime_error("A* found no guiding path.");
    std::array<double,Nodes> arc{};for(int i=1;i<Nodes;++i)arc[i]=arc[i-1]+std::hypot(path[2*i]-path[2*i-2],path[2*i+1]-path[2*i-1]);
    double length=arc.back(),peak=std::min(2.5,std::sqrt(length*.25)),ta=peak/.25,tc=std::max(0.,(length-peak*peak/.25)/peak),duration=2*ta+tc;times[body]=duration;
    for(int i=0;i<Nodes;++i) {double t=duration*i/(Nodes-1),distance,speed;
      if(t<=ta){distance=.125*t*t;speed=.25*t;}else if(t<=ta+tc){distance=.5*peak*ta+peak*(t-ta);speed=peak;}else{double left=duration-t;distance=length-.125*left*left;speed=.25*left;}
      int j=1;while(j+1<Nodes&&arc[j]<distance)++j;double ratio=(distance-arc[j-1])/std::max(1e-12,arc[j]-arc[j-1]);
      for(int axis=0;axis<2;++axis)r.states[i][4*body+axis]=path[2*(j-1)+axis]+ratio*(path[2*j+axis]-path[2*(j-1)+axis]);
      r.states[i][4*body+3]=speed;
    }
    for(int i=0;i<Nodes;++i) {int before=std::max(0,i-1),after=std::min(Nodes-1,i+1);double angle=atan2(r.states[after][4*body+1]-r.states[before][4*body+1],r.states[after][4*body]-r.states[before][4*body]);
      if(i){while(angle-r.states[i-1][4*body+2]>Pi)angle-=2*Pi;while(angle-r.states[i-1][4*body+2]<-Pi)angle+=2*Pi;}r.states[i][4*body+2]=angle;}
    r.states.front()[4*body+2]=0;r.states.back()[4*body+2]=0;
  }
  r.duration=*std::max_element(times.begin(),times.end());double dt=r.duration/(Nodes-1);
  for(auto& s:r.states){for(int b=0;b<4;++b)s[4*b+3]*=times[b]/r.duration;s[0]-=.75*cos(s[2]);s[1]-=.75*sin(s[2]);}
  auto derivative=[&](int i,int column){int a=std::max(0,i-1),b=std::min(Nodes-1,i+1);return (r.states[b][column]-r.states[a][column])/((b-a)*dt);};
  for(int i=0;i<Nodes;++i){r.states[i][17]=std::clamp(atan(1.5*derivative(i,2)/std::max(.1,r.states[i][3])),-.7,.7);r.states[i][16]=std::clamp(derivative(i,3),-.25,.25);}
  r.states.front()[17]=r.states.back()[17]=0;
  for(int i=0;i<Nodes;++i)r.states[i][18]=std::clamp(derivative(i,17),-.5,.5);
  for(int c=16;c<19;++c)r.states.front()[c]=r.states.back()[c]=0;
  return r;
}
Corridors build_corridors(const Scenario& scenario,const Trajectory& reference) {
  auto expand=[&](double x,double y){std::array<double,4> extent{};bool done[4]={};int complete=0;
    while(complete<4)for(int d=0;d<4;++d)if(!done[d]) {
      auto test=extent;test[d]+=.03;double xmin=x-extent[1],xmax=x+extent[3],ymin=y-extent[2],ymax=y+extent[0];
      if(d==0){ymin=ymax;ymax=y+test[0];}if(d==1){xmax=xmin;xmin=x-test[1];}if(d==2){ymax=ymin;ymin=y-test[2];}if(d==3){xmin=xmax;xmax=x+test[3];}
      bool valid=test[d]<=5&&xmin>=-25+sqrt(2.)&&xmax<=25-sqrt(2.)&&ymin>=-25+sqrt(2.)&&ymax<=25-sqrt(2.);
      int ax=int(ceil((xmin+25)/.5)),bx=int(ceil((xmax+25)/.5)),ay=int(ceil((ymin+25)/.5)),by=int(ceil((ymax+25)/.5));
      for(int yy=ay;valid&&yy<=by;++yy)for(int xx=ax;valid&&xx<=bx;++xx)if(xx<0||xx>100||yy<0||yy>100||scenario.occupancy[xx+101*yy])valid=false;
      if(valid)extent=test;else{done[d]=true;++complete;}
    }return extent;};
  Corridors boxes;
  for(int i=0;i<Nodes;++i)for(int b=0;b<4;++b){double x=reference.states[i][4*b],y=reference.states[i][4*b+1];if(b==0){x+=.75*cos(reference.states[i][2]);y+=.75*sin(reference.states[i][2]);}
    auto e=expand(x,y);bool found=*std::max_element(e.begin(),e.end())>0;
    for(int step=1;!found&&step<=1000;++step)for(Point delta:std::array<Point,4>{{{.01*step,0},{-.01*step,0},{0,.01*step},{0,-.01*step}}}){auto trial=expand(x+delta[0],y+delta[1]);if(*std::max_element(trial.begin(),trial.end())>0){x+=delta[0];y+=delta[1];e=trial;found=true;break;}}
    if(!found)throw std::runtime_error("Cannot construct a safe corridor.");boxes[i][b]={x-e[1],x+e[3],y-e[2],y+e[0]};
  }return boxes;
}
std::vector<CollisionFlags> activate_constraints(const Scenario& scene,const Trajectory& reference) {
  std::vector<CollisionFlags> flags;
  for(int i=0;i<Nodes;++i)for(int b=0;b<4;++b){auto state=reference.states[i];double x=state[4*b],y=state[4*b+1],angle=state[4*b+2];
    for(int o=0;o<int(scene.obstacles.size());++o){const auto& obstacle=scene.obstacles[o];bool vv[4]={},ov[4]={};
      double xmin=1e6,xmax=-1e6,ymin=1e6,ymax=-1e6;for(auto p:obstacle){xmin=std::min(xmin,p[0]);xmax=std::max(xmax,p[0]);ymin=std::min(ymin,p[1]);ymax=std::max(ymax,p[1]);}
      if(xmax<x-5.3||xmin>x+5.3||ymax<y-5.3||ymin>y+5.3)continue;
      for(int a=0;a<15;++a){double t=angle-Pi/4-.02+(Pi/2+.04)*a/14,c=cos(t),s=sin(t);
        for(int ix=0;ix<25;++ix)for(int iy=0;iy<25;++iy){double cx=x-3.05+6.10*ix/24,cy=y-3.05+6.10*iy/24;if(b==0){cx+=.75*c;cy+=.75*s;}
          double dx[]={1.05,1.05,-1.05,-1.05},dy[]={1.05,-1.05,-1.05,1.05};
          for(int v=0;v<4;++v){if(!vv[v])vv[v]=inside({cx+dx[v]*c-dy[v]*s,cy+dx[v]*s+dy[v]*c},obstacle);
            if(!ov[v]){double px=obstacle[v][0]-cx,py=obstacle[v][1]-cy;ov[v]=std::abs(px*c+py*s)<=1.05&&std::abs(-px*s+py*c)<=1.05;}}
        }
      }
      for(int v=0;v<4;++v){if(vv[v])flags.push_back({i,b,o,v,false});if(ov[v])flags.push_back({i,b,o,v,true});}
    }
  }return flags;
}
}
