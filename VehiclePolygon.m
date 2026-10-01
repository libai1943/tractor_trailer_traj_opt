function polygon=VehiclePolygon(x,y,theta,body)
% A,B,C,D corners about the wheel-axle reference point (paper Fig. 1).
if body==1, front=1.75; rear=.25; else, front=1; rear=1; end
polygon=[front,1;front,-1;-rear,-1;-rear,1];
polygon=polygon*[cos(theta),sin(theta);-sin(theta),cos(theta)]+[x,y];
end
