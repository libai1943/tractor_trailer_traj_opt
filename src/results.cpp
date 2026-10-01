#include "planner.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace trailer {
static bool overlap(const Polygon& a,const Polygon& b){for(auto polygon:{a,b})for(int i=0;i<4;++i){auto p=polygon[i],q=polygon[(i+1)%4];double ax=p[1]-q[1],ay=q[0]-p[0],n=std::hypot(ax,ay);ax/=n;ay/=n;double amin=1e9,amax=-1e9,bmin=1e9,bmax=-1e9;for(int j=0;j<4;++j){double x=a[j][0]*ax+a[j][1]*ay,y=b[j][0]*ax+b[j][1]*ay;amin=std::min(amin,x);amax=std::max(amax,x);bmin=std::min(bmin,y);bmax=std::max(bmax,y);}if(amax<=bmin+1e-8||bmax<=amin+1e-8)return false;}return true;}
double validate(const Scenario& scene,const Trajectory& r) {
  double residual=0,violation=0,dt=r.duration/(Nodes-1);auto check=[&](double v){if(!std::isfinite(v))throw std::runtime_error("Nonfinite result.");residual=std::max(residual,std::abs(v));};
  for(int i=0;i<Nodes;++i){auto s=r.states[i];for(double value:s)if(!std::isfinite(value))throw std::runtime_error("Nonfinite state.");
    violation=std::max({violation,std::abs(s[3])-2.5,std::abs(s[16])-.25,std::abs(s[17])-.7,std::abs(s[18])-.5});
    std::array<Polygon,4> polygons;
    for(int b=0;b<4;++b){int k=4*b;polygons[b]=footprint(s,b);for(auto p:polygons[b])violation=std::max({violation,std::abs(p[0])-25,std::abs(p[1])-25});
      for(auto obstacle:scene.obstacles)if(overlap(polygons[b],obstacle))throw std::runtime_error("Discrete body-obstacle collision.");
      if(b){check(s[k]-s[k-4]+3*cos(s[k+2]));check(s[k+1]-s[k-3]+3*sin(s[k+2]));check(s[k+3]-s[k-1]*cos(s[k-2]-s[k+2]));violation=std::max(violation,std::abs(s[k+2]-s[k-2])-(Pi/2-.1));}
      if(i==0||i==Nodes-1){auto point=i?scene.goal[b]:scene.start[b];check(s[k]-point[0]);check(s[k+1]-point[1]);check(s[k+2]);check(s[k+3]);}
    }
    for(int a=0;a<4;++a)for(int b=a+1;b<4;++b)if(overlap(polygons[a],polygons[b]))throw std::runtime_error("Discrete self collision.");
    if(i==0||i==Nodes-1)for(int k=16;k<19;++k)check(s[k]);
    if(i){auto previous=r.states[i-1];check(s[0]-previous[0]-dt*previous[3]*cos(previous[2]));check(s[1]-previous[1]-dt*previous[3]*sin(previous[2]));check(s[2]-previous[2]-dt*previous[3]*tan(previous[17])/1.5);check(s[3]-previous[3]-dt*previous[16]);check(s[17]-previous[17]-dt*previous[18]);for(int b=1;b<4;++b){int k=4*b;check(3*(s[k+2]-previous[k+2])-dt*previous[k-1]*sin(previous[k-2]-previous[k+2]));}}
  }
  if(residual>1e-5||violation>1e-5)throw std::runtime_error("Discrete constraints failed validation.");return residual;
}
void write_results(const Scenario& scene,const Trajectory& r,const std::string& destination,double planning,double preprocessing) {
  auto root=std::filesystem::path(destination)/("case_"+std::to_string(scene.id));std::filesystem::create_directories(root);
  std::ofstream csv(root/"trajectory.csv");csv<<std::setprecision(17)<<"time";for(int b=0;b<4;++b)for(auto name:{"x","y","theta","v"})csv<<","<<name<<b;csv<<",a,phi,omega\n";
  for(int i=0;i<Nodes;++i){csv<<r.duration*i/(Nodes-1);for(double v:r.states[i])csv<<","<<v;csv<<"\n";}
  std::ofstream report(root/"validation.json");report<<std::setprecision(17)<<"{\"case_id\":"<<scene.id<<",\"duration\":"<<r.duration<<",\"planning_seconds\":"<<planning<<",\"preprocessing_seconds\":"<<preprocessing<<",\"dynamics_residual\":"<<validate(scene,r)<<",\"configuration_points\":101,\"collision_pairs\":0,\"passed\":true}\n";
  const char* colors[]={"#df2530","#1677c8","#19944b","#fa8917"};
  std::ofstream image(root/"trajectory.svg");image<<"<svg xmlns='http://www.w3.org/2000/svg' width='850' height='900' viewBox='0 0 850 900'><rect width='850' height='900' fill='white'/><style>text{font-family:Arial;font-size:18px}</style>";
  image<<"<text x='425' y='30' text-anchor='middle'>Chapter 9 | Case "<<scene.id<<" | T = "<<std::fixed<<std::setprecision(3)<<r.duration<<" s</text>";
  auto polygon=[&](Polygon p,const char* color,const char* fill,double opacity){image<<"<polygon points='";for(auto point:p)image<<50+(point[0]+25)*15<<","<<815-(point[1]+25)*15<<" ";image<<"' fill='"<<fill<<"' stroke='"<<color<<"' stroke-width='0.7' opacity='"<<opacity<<"'/>";};
  image<<"<defs><clipPath id='map'><rect x='50' y='65' width='750' height='750'/></clipPath></defs><g clip-path='url(#map)'>";
  for(auto p:scene.obstacles)polygon(p,"#747e85","#747e85",1);
  for(auto s:r.states)for(int b=0;b<4;++b)polygon(footprint(s,b),colors[b],"none",.45);
  for(int b=0;b<4;++b){image<<"<polyline fill='none' stroke='"<<colors[b]<<"' stroke-width='2.5' points='";for(auto s:r.states)image<<50+(s[4*b]+25)*15<<","<<815-(s[4*b+1]+25)*15<<" ";image<<"'/>";}
  image<<"</g><rect x='50' y='65' width='750' height='750' fill='none' stroke='black'/><text x='425' y='862' text-anchor='middle'>x (m)</text><text transform='translate(18 440) rotate(-90)' text-anchor='middle'>y (m)</text>";
  for(int value=-25;value<=25;value+=5){image<<"<text x='"<<50+(value+25)*15<<"' y='840' text-anchor='middle' font-size='12'>"<<value<<"</text><text x='44' y='"<<820-(value+25)*15<<"' text-anchor='end' font-size='12'>"<<value<<"</text>";}
  for(int b=0;b<4;++b)image<<"<text x='"<<70+b*190<<"' y='892' fill='"<<colors[b]<<"'>"<<(b?"Trailer "+std::to_string(b):"Tractor")<<"</text>";image<<"</svg>";
  std::ofstream profiles(root/"profiles.svg");profiles<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='850'><rect width='1200' height='850' fill='white'/><style>text{font:16px Arial}</style>";
  const char* labels[]={"Speed (m/s)","Heading (rad)","Acceleration (m/s^2)","Steering angle (rad)","Steering rate (rad/s)","Articulation angle (rad)"};double lo[]={-2.6,-.5,-.3,-.8,-.55,-1.6},hi[]={2.6,3.6,.3,.8,.55,1.6};
  for(int panel=0;panel<6;++panel){double left=70+(panel%2)*590,top=45+(panel/2)*275,width=480,height=195;profiles<<"<text x='"<<left<<"' y='"<<top-15<<"'>"<<labels[panel]<<"</text><rect x='"<<left<<"' y='"<<top<<"' width='"<<width<<"' height='"<<height<<"' fill='none' stroke='#999'/>";
    int curves=panel<2?4:(panel==5?3:1);for(int b=0;b<curves;++b){profiles<<"<polyline fill='none' stroke='"<<colors[b]<<"' stroke-width='1.7' points='";for(int i=0;i<Nodes;++i){auto s=r.states[i];double v=panel==0?s[4*b+3]:panel==1?s[4*b+2]:panel==2?s[16]:panel==3?s[17]:panel==4?s[18]:s[4*b+2]-s[4*b+6];profiles<<left+width*i/(Nodes-1)<<","<<top+height*(hi[panel]-v)/(hi[panel]-lo[panel])<<" ";}profiles<<"'/>";}
    profiles<<"<text x='"<<left<<"' y='"<<top+height+23<<"'>0</text><text x='"<<left+width-25<<"' y='"<<top+height+23<<"'>"<<std::setprecision(4)<<r.duration<<"</text><text x='"<<left+width/2-35<<"' y='"<<top+height+23<<"'>Time (s)</text>";
    for(int tick=0;tick<=4;++tick)profiles<<"<text x='"<<left-8<<"' y='"<<top+height*tick/4+5<<"' text-anchor='end'>"<<std::setprecision(3)<<hi[panel]-(hi[panel]-lo[panel])*tick/4<<"</text>";
  }profiles<<"</svg>";
}
}
