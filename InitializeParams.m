function InitializeParams()
global params
params.xmin = -25;
params.xmax = 25;
params.ymin = -25;
params.ymax = 25;
params.xhorizon = params.xmax - params.xmin;
params.yhorizon = params.ymax - params.ymin;
params.L_tractor_front_hang = 0.25;
params.L_tractor_wheelbase = 1.5;
params.L_tractor_rear_hang = 0.25;
params.L_half_width = 1;
params.vehicle_width = 2 * params.L_half_width;
params.M = [0, 0, 0, 0];
params.L = [3.0, 3.0, 3.0, 3.0];
params.L_trailer_front_hang = 1;
params.L_trailer_rear_hang = 1;
params.tractor_vehicle_length = params.L_tractor_wheelbase + params.L_tractor_front_hang + params.L_tractor_rear_hang;
params.trailer_vehicle_length = params.L_trailer_front_hang + params.L_trailer_rear_hang;
params.radius = max(sqrt(2) * params.L_half_width, hypot(params.L_half_width, 0.5 * params.tractor_vehicle_length));
params.v_max = 2.5;
params.phy_max = 0.7;
params.a_max = 0.25;
params.w_max = 0.5;
[params.x2_init, params.y2_init, params.x3_init, params.y3_init, params.x4_init, params.y4_init] = MapFrom6DimTo4Dim(params.x1_init, params.y1_init, params.theta1_init, params.theta2_init, params.theta3_init, params.theta4_init);
[params.x2_end_norm, params.y2_end_norm, params.x3_end_norm, params.y3_end_norm, params.x4_end_norm, params.y4_end_norm] = MapFrom6DimTo4Dim(params.x1_end_norm, params.y1_end_norm, params.theta1_end_norm, params.theta2_end_norm, params.theta3_end_norm, params.theta4_end_norm);
params.dx = 0.5;
params.dy = 0.5;
params.nx = ceil(params.xhorizon / params.dx) + 1;
params.ny = ceil(params.yhorizon / params.dy) + 1;
params.NFE = 100; % Paper Table I: finite-element intervals
params.nfe = params.NFE+1;
params.unit_step = 0.03;
params.max_step = 5;


params.colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255;
params.max_iter = 5;
params.max_iter_trmo = 10;
params.feasibility_tolerance = 1e-3;
params.cost_tolerance = 1.0;
params.weight_inf_degree_penalty = 10000;
params.trust_region_scale = 3;
params.trust_region_arc_length = 0.25 * pi;
params.trial_grids = FormRelativeTrialGrids();
WriteFilesForNLPSetting();
WriteObsFile();
end

function [x2, y2, x3, y3, x4, y4] = MapFrom6DimTo4Dim(x1, y1, theta1, theta2, theta3, theta4)
global params
x2 = x1 - params.L(2) * cos(theta2) - params.M(1) * cos(theta1);
y2 = y1 - params.L(2) * sin(theta2) - params.M(1) * sin(theta1);
x3 = x2 - params.L(3) * cos(theta3) - params.M(2) * cos(theta2);
y3 = y2 - params.L(3) * sin(theta3) - params.M(2) * sin(theta2);
x4 = x3 - params.L(4) * cos(theta4) - params.M(3) * cos(theta3);
y4 = y3 - params.L(4) * sin(theta4) - params.M(3) * sin(theta3);
end

function WriteFilesForNLPSetting()
global params
if isfile('M'), delete('M'); end
fid = fopen('M', 'w');
fprintf(fid, '1  %.17g \r\n', params.M(1));
fprintf(fid, '2  %.17g \r\n', params.M(2));
fprintf(fid, '3  %.17g \r\n', params.M(3));
fprintf(fid, '4  %.17g \r\n', params.M(4));
fclose(fid);

if isfile('BV'), delete('BV'); end
fid = fopen('BV', 'w');

fprintf(fid, '1  1  %.17g \r\n', params.x1_init);
fprintf(fid, '1  2  %.17g \r\n', params.y1_init);
fprintf(fid, '1  3  %.17g \r\n', params.theta1_init);
fprintf(fid, '1  4  %.17g \r\n', params.x1_end_norm);
fprintf(fid, '1  5  %.17g \r\n', params.y1_end_norm);
fprintf(fid, '1  6  %.17g \r\n', params.theta1_end_norm);

fprintf(fid, '2  1  %.17g \r\n', params.x2_init);
fprintf(fid, '2  2  %.17g \r\n', params.y2_init);
fprintf(fid, '2  3  %.17g \r\n', params.theta2_init);
fprintf(fid, '2  4  %.17g \r\n', params.x2_end_norm);
fprintf(fid, '2  5  %.17g \r\n', params.y2_end_norm);
fprintf(fid, '2  6  %.17g \r\n', params.theta2_end_norm);

fprintf(fid, '3  1  %.17g \r\n', params.x3_init);
fprintf(fid, '3  2  %.17g \r\n', params.y3_init);
fprintf(fid, '3  3  %.17g \r\n', params.theta3_init);
fprintf(fid, '3  4  %.17g \r\n', params.x3_end_norm);
fprintf(fid, '3  5  %.17g \r\n', params.y3_end_norm);
fprintf(fid, '3  6  %.17g \r\n', params.theta3_end_norm);

fprintf(fid, '4  1  %.17g \r\n', params.x4_init);
fprintf(fid, '4  2  %.17g \r\n', params.y4_init);
fprintf(fid, '4  3  %.17g \r\n', params.theta4_init);
fprintf(fid, '4  4  %.17g \r\n', params.x4_end_norm);
fprintf(fid, '4  5  %.17g \r\n', params.y4_end_norm);
fprintf(fid, '4  6  %.17g \r\n', params.theta4_end_norm);
fclose(fid);

if isfile('param_vector'), delete('param_vector'); end
fid = fopen('param_vector', 'w');
fprintf(fid,'1 %g \r\n', params.nfe);
fprintf(fid,'2 %g \r\n', params.weight_inf_degree_penalty);
fprintf(fid,'3 %g \r\n', params.trust_region_scale);
fprintf(fid,'4 %g \r\n', 0);
fprintf(fid,'5 %g \r\n', 0);
fclose(fid);
end

function WriteObsFile()
global params
if isfile('Current_vertex'), delete('Current_vertex'); end
fid = fopen('Current_vertex', 'w');
for ii = 1 : params.Nobs
    cur_obs = params.obs{1, ii};
    x = cur_obs.x;
    y = cur_obs.y;
    for jj = 1 : 4
        fprintf(fid, '%g %g %g %.17g \r\n', ii, jj, 1, x(jj));
        fprintf(fid, '%g %g %g %.17g \r\n', ii, jj, 2, y(jj));
    end
end
fclose(fid);

if isfile('Number_obstacle'), delete('Number_obstacle'); end
fid = fopen('Number_obstacle', 'w');
fprintf(fid,' %g\r\n', params.Nobs);
fclose(fid);

if isfile('Area'), delete('Area'); end
fid = fopen('Area', 'w');
for ii = 1 : params.Nobs
    area = CalculatePolygonArea(params.obs{1, ii});
    fprintf(fid,'%g %g\r\n', ii, min([area + 0.1, area * 1.01]));
end
fclose(fid);
end

function area = CalculatePolygonArea(V)
len = length(V.x) - 1;
area = 0;
X = V.x;
Y = V.y;
for ii = 1 : len
    v1x = X(ii);
    v1y = Y(ii);
    v2x = X(ii+1);
    v2y = Y(ii+1);
    area = area + v1x * v2y - v1y * v2x;
end
area = 0.5 * abs(area);
end

function trial_grids = FormRelativeTrialGrids()
global params
num_grids1 = 10 + round(params.trust_region_scale / 0.2);
num_grids2 = 15;
ds = linspace(-params.trust_region_scale - 0.05, params.trust_region_scale + 0.05, num_grids1);
dt = linspace(-params.trust_region_arc_length - 0.02, params.trust_region_arc_length + 0.02, num_grids2);
vec = [];
for ii = 1 : num_grids1
    for jj = 1 : num_grids1
        for kk = 1 : num_grids2
            vec = [vec; ds(ii), ds(jj), dt(kk)];
        end
    end
end
trial_grids.dx = vec(:,1)';
trial_grids.dy = vec(:,2)';
trial_grids.dtheta = vec(:,3)';
end
