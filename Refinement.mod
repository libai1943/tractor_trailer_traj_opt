param Nv := 4;
param param_vector{i in {1..5}};
param Nfe := param_vector[1];
param trust_region_length := param_vector[3];
param MaximumTf := param_vector[4];
param trust_region_arc_length := param_vector[5];
var tf >= 0.1;
var dt = tf / (Nfe - 1);
set I := {1..Nfe};
param M{i in {1..Nv}};
param BV{i in {1..Nv}, j in {1..6}};
param VXY{i in {1..Nv}, j in {1..Nfe}, k in {1..3}};

param Nobs;
param BI4VV{i in {1..Nobs}, j in {1..Nv}, k in {1..Nfe}, m in {1..4}};
param BI4OV{i in {1..Nobs}, j in {1..4}, k in {1..Nv}, m in {1..Nfe}};
param PPP{i in {1..Nobs}, j in {1..4}, k in {1..2}};
param ObstacleVertexList{i in {1..Nobs}}; 
param AREA{i in {1..Nobs}};

param a_max := 0.25;
param v_max := 2.5;
param w_max := 0.5;
param phy_max := 0.7;

param L_tractor_front_hang := 0.25;
param L_tractor_rear_hang := 0.25;
param L_tractor_wheelbase := 1.5;
param LHW := 1;
param L := 3.0;
param L_trailer_front_hang := 1;
param L_trailer_rear_hang := 1;

var x{i in I, k in {1..Nv}};
var y{i in I, k in {1..Nv}};
var theta{i in I, k in {1..Nv}};
var v{i in I, k in {1..Nv}};
var phy{i in I};
var a{i in I};
var w{i in I};
var AX{i in I, k in {1..Nv}};
var BX{i in I, k in {1..Nv}};
var CX{i in I, k in {1..Nv}};
var DX{i in I, k in {1..Nv}};
var AY{i in I, k in {1..Nv}};
var BY{i in I, k in {1..Nv}};
var CY{i in I, k in {1..Nv}};
var DY{i in I, k in {1..Nv}};

minimize objective_:
tf;

s.t. Timer:
tf <= MaximumTf;


s.t. box_on_x {i in {1..Nv}, j in {1..Nfe}}:
VXY[i,j,1] - trust_region_length <= x[j,i] <= VXY[i,j,1] + trust_region_length;
s.t. box_on_y {i in {1..Nv}, j in {1..Nfe}}:
VXY[i,j,2] - trust_region_length <= y[j,i] <= VXY[i,j,2] + trust_region_length;
s.t. box_on_theta {i in {1..Nv}, j in {1..Nfe}}:
VXY[i,j,3] - trust_region_arc_length <= theta[j,i] <= VXY[i,j,3] + trust_region_arc_length;

s.t. DIFF_dxdt {i in {2..Nfe}}:
x[i,1] = x[i-1,1] + dt * v[i-1,1] * cos(theta[i-1,1]);

s.t. DIFF_dydt {i in {2..Nfe}}:
y[i,1] = y[i-1,1] + dt * v[i-1,1] * sin(theta[i-1,1]);

s.t. DIFF_dvdt {i in {2..Nfe}}:
v[i,1] = v[i-1,1] + dt * a[i-1];

s.t. DIFF_dtheta1dt {i in {2..Nfe}}:
theta[i,1] = theta[i-1,1] + dt * v[i-1,1] * tan(phy[i-1]) / L_tractor_wheelbase;

s.t. DIFF_dphydt {i in {2..Nfe}}:
phy[i] = phy[i-1] + dt * w[i-1];

s.t. ALGE_x_2_to_Nv {i in I, j in {2..Nv}}:
x[i,j] = x[i,j-1] - L * cos(theta[i,j]) - M[j-1] * cos(theta[i,j-1]);

s.t. ALGE_y_2_to_Nv {i in I, j in {2..Nv}}:
y[i,j] = y[i,j-1] - L * sin(theta[i,j]) - M[j-1] * sin(theta[i,j-1]);

s.t. DIFF_theta_2_to_Nv {i in {2..Nfe}, j in {2..Nv}}:
L * (theta[i,j] - theta[i-1,j]) = dt * v[i-1,j-1] * sin(theta[i-1,j-1] - theta[i-1,j]);

s.t. ALG_v_2_to_Nv {i in I, j in {2..Nv}}:
v[i,j] = v[i,j-1] * cos(theta[i,j-1] - theta[i,j]);

s.t. RELATIONSHIP_AX1 {i in I}:
AX[i,1] = x[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * cos(theta[i,1]) - LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_BX1 {i in I}:
BX[i,1] = x[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * cos(theta[i,1]) + LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_CX1 {i in I}:
CX[i,1] = x[i,1] - L_tractor_rear_hang * cos(theta[i,1]) + LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_DX1 {i in I}:
DX[i,1] = x[i,1] - L_tractor_rear_hang * cos(theta[i,1]) - LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_AY1 {i in I}:
AY[i,1] = y[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * sin(theta[i,1]) + LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_BY1 {i in I}:
BY[i,1] = y[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * sin(theta[i,1]) - LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_CY1 {i in I}:
CY[i,1] = y[i,1] - L_tractor_rear_hang * sin(theta[i,1]) - LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_DY1 {i in I}:
DY[i,1] = y[i,1] - L_tractor_rear_hang * sin(theta[i,1]) + LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_AX2toNv {i in I, k in {2..Nv}}:
AX[i,k] = x[i,k] + L_trailer_front_hang * cos(theta[i,k]) - LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_BX2toN {i in I, k in {2..Nv}}:
BX[i,k] = x[i,k] + L_trailer_front_hang * cos(theta[i,k]) + LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_CX2toN {i in I, k in {2..Nv}}:
CX[i,k] = x[i,k] - L_trailer_rear_hang * cos(theta[i,k]) + LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_DX2toN {i in I, k in {2..Nv}}:
DX[i,k] = x[i,k] - L_trailer_rear_hang * cos(theta[i,k]) - LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_AY2toN {i in I, k in {2..Nv}}:
AY[i,k] = y[i,k] + L_trailer_front_hang * sin(theta[i,k]) + LHW * cos(theta[i,k]);

s.t. RELATIONSHIP_BY2toN {i in I, k in {2..Nv}}:
BY[i,k] = y[i,k] + L_trailer_front_hang * sin(theta[i,k]) - LHW * cos(theta[i,k]);

s.t. RELATIONSHIP_CY2toN {i in I, k in {2..Nv}}:
CY[i,k] = y[i,k] - L_trailer_rear_hang * sin(theta[i,k]) - LHW * cos(theta[i,k]);

s.t. RELATIONSHIP_DY2toN {i in I, k in {2..Nv}}:
DY[i,k] = y[i,k] - L_trailer_rear_hang * sin(theta[i,k]) + LHW * cos(theta[i,k]);


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
s.t. End_X1:
x[Nfe,1] = BV[1,4];
s.t. End_Y1:
y[Nfe,1] = BV[1,5];
s.t. End_Theta {i in {1..Nv}}:
theta[Nfe,i] = BV[i,6];
s.t. Init_v:
v[1,1] = 0;
s.t. End_v:
v[Nfe,1] = 0;

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

s.t. Bonds_AX {i in I, j in {1..Nv}}:
-25 <= AX[i,j] <= 25;
s.t. Bonds_BX {i in I, j in {1..Nv}}:
-25 <= BX[i,j] <= 25;
s.t. Bonds_CX {i in I, j in {1..Nv}}:
-25 <= CX[i,j] <= 25;
s.t. Bonds_DX {i in I, j in {1..Nv}}:
-25 <= DX[i,j] <= 25;
s.t. Bonds_AY {i in I, j in {1..Nv}}:
-25 <= AY[i,j] <= 25;
s.t. Bonds_BY {i in I, j in {1..Nv}}:
-25 <= BY[i,j] <= 25;
s.t. Bonds_CY {i in I, j in {1..Nv}}:
-25 <= CY[i,j] <= 25;
s.t. Bonds_DY {i in I, j in {1..Nv}}:
-25 <= DY[i,j] <= 25;

## Collison avoidance ##
s.t. eq_PPPoutsideABCD {i in I, pp in {1..Nv}, nn in {1..Nobs}, jj in {1..4}}:
BI4OV[nn,jj,pp,i] * (abs((AX[i,pp] - PPP[nn,jj,1])*(BY[i,pp] - PPP[nn,jj,2]) - (AY[i,pp] - PPP[nn,jj,2])*(BX[i,pp] - PPP[nn,jj,1])) * 0.5 + abs((BX[i,pp] - PPP[nn,jj,1])*(CY[i,pp] - PPP[nn,jj,2]) - (BY[i,pp] - PPP[nn,jj,2])*(CX[i,pp] - PPP[nn,jj,1])) * 0.5 + abs((CX[i,pp] - PPP[nn,jj,1])*(DY[i,pp] - PPP[nn,jj,2]) - (CY[i,pp] - PPP[nn,jj,2])*(DX[i,pp] - PPP[nn,jj,1])) * 0.5 + abs((DX[i,pp] - PPP[nn,jj,1])*(AY[i,pp] - PPP[nn,jj,2]) - (DY[i,pp] - PPP[nn,jj,2])*(AX[i,pp] - PPP[nn,jj,1])) * 0.5) >= BI4OV[nn,jj,pp,i] * (4.1);

s.t. eq_AoutsidePRECTANGLEPPP {i in I, pp in {1..Nv}, nn in {1..Nobs}}:
BI4VV[nn,pp,i,1] * (abs((PPP[nn,1,1] - AX[i,pp])*( PPP[nn,2,2] - AY[i,pp]) - (PPP[nn,1,2] - AY[i,pp])*(PPP[nn,2,1] - AX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - AX[i,pp])*(PPP[nn,3,2] - AY[i,pp]) - (PPP[nn,2,2] - AY[i,pp])*( PPP[nn,3,1] - AX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - AX[i,pp])*( PPP[nn,4,2] - AY[i,pp]) - (PPP[nn,3,2] - AY[i,pp])*( PPP[nn,4,1] - AX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - AX[i,pp])*( PPP[nn,1,2] - AY[i,pp]) - (PPP[nn,4,2] - AY[i,pp])*( PPP[nn,1,1] - AX[i,pp])) * 0.5) >= BI4VV[nn,pp,i,1] * AREA[nn];

s.t. eq_BoutsidePRECTANGLEPPP {i in I, pp in {1..Nv}, nn in {1..Nobs}}:
BI4VV[nn,pp,i,2] * (abs((PPP[nn,1,1] - BX[i,pp])*( PPP[nn,2,2] - BY[i,pp]) - (PPP[nn,1,2] - BY[i,pp])*(PPP[nn,2,1] - BX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - BX[i,pp])*(PPP[nn,3,2] - BY[i,pp]) - (PPP[nn,2,2] - BY[i,pp])*( PPP[nn,3,1] - BX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - BX[i,pp])*( PPP[nn,4,2] - BY[i,pp]) - (PPP[nn,3,2] - BY[i,pp])*( PPP[nn,4,1] - BX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - BX[i,pp])*( PPP[nn,1,2] - BY[i,pp]) - (PPP[nn,4,2] - BY[i,pp])*( PPP[nn,1,1] - BX[i,pp])) * 0.5) >= BI4VV[nn,pp,i,2] * AREA[nn];

s.t. eq_CoutsidePRECTANGLEPPP {i in I, pp in {1..Nv}, nn in {1..Nobs}}:
BI4VV[nn,pp,i,3] * (abs((PPP[nn,1,1] - CX[i,pp])*( PPP[nn,2,2] - CY[i,pp]) - (PPP[nn,1,2] - CY[i,pp])*(PPP[nn,2,1] - CX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - CX[i,pp])*(PPP[nn,3,2] - CY[i,pp]) - (PPP[nn,2,2] - CY[i,pp])*( PPP[nn,3,1] - CX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - CX[i,pp])*( PPP[nn,4,2] - CY[i,pp]) - (PPP[nn,3,2] - CY[i,pp])*( PPP[nn,4,1] - CX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - CX[i,pp])*( PPP[nn,1,2] - CY[i,pp]) - (PPP[nn,4,2] - CY[i,pp])*( PPP[nn,1,1] - CX[i,pp])) * 0.5) >= BI4VV[nn,pp,i,3] * AREA[nn];

s.t. eq_DoutsidePRECTANGLEPPP {i in I, pp in {1..Nv}, nn in {1..Nobs}}:
BI4VV[nn,pp,i,4] * (abs((PPP[nn,1,1] - DX[i,pp])*( PPP[nn,2,2] - DY[i,pp]) - (PPP[nn,1,2] - DY[i,pp])*(PPP[nn,2,1] - DX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - DX[i,pp])*(PPP[nn,3,2] - DY[i,pp]) - (PPP[nn,2,2] - DY[i,pp])*( PPP[nn,3,1] - DX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - DX[i,pp])*( PPP[nn,4,2] - DY[i,pp]) - (PPP[nn,3,2] - DY[i,pp])*( PPP[nn,4,1] - DX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - DX[i,pp])*( PPP[nn,1,2] - DY[i,pp]) - (PPP[nn,4,2] - DY[i,pp])*( PPP[nn,1,1] - DX[i,pp])) * 0.5) >= BI4VV[nn,pp,i,4] * AREA[nn];

data;
param: param_vector := include param_vector;
param: M := include M;
param: BV := include BV;
param: VXY := include VXY;
param Nobs := include Number_obstacle;
param: PPP := include Current_vertex;
param AREA := include Area;
param BI4VV := include BI4VV;
param BI4OV := include BI4OV;