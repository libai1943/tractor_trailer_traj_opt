function r=WriteInitialGuess(paths)
% Rest-to-rest time-optimal scalar profiles (PMP), attached to four coarse paths.
global params
n=params.nfe; r.x=zeros(n,4); r.y=r.x; r.theta=r.x; r.v=r.x;
durations=zeros(1,4);
for body=1:4
    p=paths.(sprintf('path%d',body));
    s=[0,cumsum(hypot(diff(p.x),diff(p.y)))];
    [s,indices]=unique(s,'stable'); x=p.x(indices); y=p.y(indices);
    length_path=s(end); acceleration=params.a_max;
    peak=min(params.v_max,sqrt(length_path*acceleration));
    ta=peak/acceleration; tc=max(0,(length_path-peak^2/acceleration)/peak);
    duration=2*ta+tc; durations(body)=duration;
    t=linspace(0,duration,n)'; distance=zeros(n,1); speed=distance;
    for i=1:n
        if t(i)<=ta
            distance(i)=.5*acceleration*t(i)^2; speed(i)=acceleration*t(i);
        elseif t(i)<=ta+tc
            distance(i)=.5*peak*ta+peak*(t(i)-ta); speed(i)=peak;
        else
            remaining=duration-t(i); distance(i)=length_path-.5*acceleration*remaining^2;
            speed(i)=acceleration*remaining;
        end
    end
    cx=interp1(s,x,min(length_path,max(0,distance))); cy=interp1(s,y,min(length_path,max(0,distance)));
    angle=unwrap(atan2(gradient(cy),gradient(cx)));
    angle(1)=params.(sprintf('theta%d_init',body));
    angle(end)=params.(sprintf('theta%d_end_norm',body));
    r.x(:,body)=cx; r.y(:,body)=cy; r.theta(:,body)=angle; r.v(:,body)=speed;
end
r.tf=max(durations);
r.v=r.v.*(durations/r.tf);
r.x(:,1)=r.x(:,1)-.75*cos(r.theta(:,1));
r.y(:,1)=r.y(:,1)-.75*sin(r.theta(:,1));
dt=r.tf/(n-1);
r.phy=atan(params.L_tractor_wheelbase*gradient(r.theta(:,1),dt)./max(.1,r.v(:,1)));
r.phy=max(-params.phy_max,min(params.phy_max,r.phy)); r.phy([1,end])=0;
r.a=max(-params.a_max,min(params.a_max,gradient(r.v(:,1),dt)));
r.w=max(-params.w_max,min(params.w_max,gradient(r.phy,dt)));
r.a([1,end])=0; r.w([1,end])=0;
end
