param Nv := 4;
var tf >= 0.1;
param param_vector{i in {1..5}};
param Nfe := param_vector[1];
param w_penalty := param_vector[2];
var dt = tf / (Nfe - 1);
set I := {1..Nfe};
param M{i in {1..Nv}};
param BV{i in {1..Nv}, j in {1..6}};
param AABB{i in I, j in {1..16}};

param a_max := 0.25;
param v_max := 2.5;
param w_max := 0.5;
param phy_max := 0.7;

param L_tractor_front_hang := 0.25;
param L_tractor_rear_hang := 0.25;
param L_tractor_wheelbase := 1.5;
param L := 3.0;

var x{i in I, k in {1..Nv}};
var y{i in I, k in {1..Nv}};
var xc{i in I};
var yc{i in I};
var theta{i in I, k in {1..Nv}};
var v{i in I, k in {1..Nv}};
var phy{i in I};
var a{i in I};
var w{i in I};


minimize objective_: tf + w_penalty*((sum{i in {2..Nfe}}((x[i,1] - x[i-1,1] - dt * v[i-1,1] * cos(theta[i-1,1]))^2 + (y[i,1] - y[i-1,1] - dt * v[i-1,1] * sin(theta[i-1,1]))^2 + (v[i,1] - v[i-1,1] - dt * a[i-1])^2 + (theta[i,1] - theta[i-1,1] - dt * v[i-1,1] * tan(phy[i-1]) / L_tractor_wheelbase)^2 + (phy[i] - phy[i-1] - dt * w[i-1])^2) + sum{i in I}((x[i,1] + ((L_tractor_front_hang + L_tractor_rear_hang + L_tractor_wheelbase) / 2 - L_tractor_rear_hang) * cos(theta[i,1]) - xc[i])^2 + (y[i,1] + ((L_tractor_front_hang + L_tractor_rear_hang + L_tractor_wheelbase) / 2 - L_tractor_rear_hang) * sin(theta[i,1]) - yc[i])^2) + sum{i in I, j in {2..Nv}}((x[i,j] - x[i,j-1] + L * cos(theta[i,j]) + M[j-1] * cos(theta[i,j-1]))^2 + (y[i,j] - y[i,j-1] + L * sin(theta[i,j]) + M[j-1] * sin(theta[i,j-1]))^2) + sum{i in {2..Nfe}, j in {2..Nv}}((L * (theta[i,j] - theta[i-1,j]) - dt * v[i-1,j-1] * sin(theta[i-1,j-1] - theta[i-1,j]))^2 + (v[i,j] - v[i,j-1] * cos(theta[i,j-1] - theta[i,j]))^2)));

s.t. timer:
tf <= 100;

s.t. Init_X {i in {1..1}}:
x[1,i] = BV[i,1];
s.t. Init_Y {i in {1..1}}:
y[1,i] = BV[i,2];
s.t. Init_Theta {i in {1..Nv}}:
theta[1,i] = BV[i,3];
s.t. Init_W:
w[1] = 0;
s.t. Init_A:
a[1] = 0;
s.t. Init_Phy:
phy[1] = 0;
s.t. End_W:
w[Nfe] = 0;
s.t. End_A:
a[Nfe] = 0;
s.t. End_Phy:
phy[Nfe] = 0;
s.t. Init_v:
v[1,1] = 0;
s.t. End_v:
v[Nfe,1] = 0;

s.t. End_X1:
x[Nfe,1] = BV[1,4];
s.t. End_Y1:
y[Nfe,1] = BV[1,5];
s.t. End_Theta {i in {1..Nv}}:
theta[Nfe,i] = BV[i,6];


s.t. Tractor_radius_x {i in I}:
AABB[i,1] <= xc[i] <= AABB[i,2];
s.t. Trailer_radius_y {i in I}:
AABB[i,3] <= yc[i] <= AABB[i,4];
s.t. Tractor1_radius_x {i in I}:
AABB[i,5] <= x[i,2] <= AABB[i,6];
s.t. Tractor1_radius_y {i in I}:
AABB[i,7] <= y[i,2] <= AABB[i,8];
s.t. Tractor2_radius_x {i in I}:
AABB[i,9] <= x[i,3] <= AABB[i,10];
s.t. Tractor2_radius_y {i in I}:
AABB[i,11] <= y[i,3] <= AABB[i,12];
s.t. Tractor3_radius_x {i in I}:
AABB[i,13] <= x[i,4] <= AABB[i,14];
s.t. Tractor3_radius_y {i in I}:
AABB[i,15] <= y[i,4] <= AABB[i,16];

s.t. Bonds_v {i in I}:
-v_max <= v[i,1] <= v_max;
s.t. Bonds_a {i in I}:
-a_max <= a[i] <= a_max;
s.t. Bonds_phy {i in I}:
-phy_max <= phy[i] <= phy_max;
s.t. Bonds_w {i in I}:
-w_max <= w[i] <= w_max;
s.t. Bonds_dtheta {i in I, k in {2..Nv}}:
-(4*atan(1)/2) + 0.1 <= theta[i,k] - theta[i,k-1] <= (4*atan(1)/2) - 0.1;

s.t. Bonds_xxxx2 {i in I, j in {2..Nv}}:
-23.5858 <= x[i,j] <= 23.5858;
s.t. Bonds_yyyy2 {i in I, j in {2..Nv}}:
-23.5858 <= y[i,j] <= 23.5858;
s.t. Bonds_xxxx1 {i in I}:
-23.5858 <= x[i,1] <= 23.5858;
s.t. Bonds_yyyy1 {i in I}:
-23.5858 <= y[i,1] <= 23.5858;

data;
param: param_vector := include param_vector;
param: M := include M;
param: BV := include BV;
param: AABB := include AABB;