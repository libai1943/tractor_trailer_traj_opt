function UpdateNlpFormulation()
global params
x=readmatrix('x.txt'); y=readmatrix('y.txt'); theta=readmatrix('theta.txt');
Nfe = params.nfe;
Nv = 4;
x = reshape(x, Nfe, Nv)';
y = reshape(y, Nfe, Nv)';
theta = reshape(theta, Nfe, Nv)';
if isfile('VXY'), delete('VXY'); end
fid = fopen('VXY', 'w');
for ii = 1 : Nv
    for jj = 1 : params.nfe
        fprintf(fid, '%g %g 1 %.17g \r\n', ii, jj, x(ii,jj));
        fprintf(fid, '%g %g 2 %.17g \r\n', ii, jj, y(ii,jj));
        fprintf(fid, '%g %g 3 %.17g \r\n', ii, jj, theta(ii,jj));
    end
end
fclose(fid);

Nfe = params.nfe;
Nv = 4;
Nobs = params.Nobs;
params.mat_vv = zeros(Nobs, Nv, Nfe, 4);
params.mat_ov = zeros(Nobs, 4, Nv, Nfe);
basic_param_1 = 0.5 * (params.L_tractor_front_hang + params.L_tractor_wheelbase - params.L_tractor_rear_hang);
for ii = 1 : params.nfe
    for jj = 1 : Nv
        if (jj == 1)
            xx = x(jj, ii) + basic_param_1 * cos(theta(jj, ii));
            yy = y(jj, ii) + basic_param_1 * sin(theta(jj, ii));
        else
            xx = x(jj, ii);
            yy = y(jj, ii);
        end
        tt = theta(jj, ii);
        EvaluateCollisionRisks(xx, yy, tt, ii, jj);
    end
end

if isfile('BI4VV'), delete('BI4VV'); end
fid = fopen('BI4VV', 'w');
for ii = 1 : Nobs
    for jj = 1 : Nv
        for kk = 1 : Nfe
            for mm = 1 : 4
                fprintf(fid, '%g %g %g %g %g\r\n', ii, jj, kk, mm, params.mat_vv(ii,jj,kk,mm));
            end
        end
    end
end
fclose(fid);

if isfile('BI4OV'), delete('BI4OV'); end
fid = fopen('BI4OV', 'w');
for ii = 1 : Nobs
    for jj = 1 : 4
        for kk = 1 : Nv
            for mm = 1 : Nfe
                fprintf(fid, '%g %g %g %g %g\r\n', ii, jj, kk, mm, params.mat_ov(ii,jj,kk,mm));
            end
        end
    end
end
fclose(fid);

terminal_time=readmatrix('terminal_time.txt');
if isfile('param_vector'), delete('param_vector'); end
fid = fopen('param_vector', 'w');
fprintf(fid,'1 %g \r\n', params.nfe);
fprintf(fid,'2 %g \r\n', -999);
fprintf(fid,'3 %g \r\n', params.trust_region_scale);
fprintf(fid,'4 %g \r\n', max(100,ceil(terminal_time)));
fprintf(fid,'5 %g \r\n', params.trust_region_arc_length);
fclose(fid);
end

function EvaluateCollisionRisks(xx, yy, tt, ii, jj)
global params
x = params.trial_grids.dx + xx;
y = params.trial_grids.dy + yy;
t = params.trial_grids.dtheta + tt;
if jj==1
    x=x+0.75*(cos(t)-cos(tt));
    y=y+0.75*(sin(t)-sin(tt));
end

buffer_1 = 0.05;
AX = x + (params.L_trailer_front_hang + buffer_1) * cos(t) - (params.L_half_width + buffer_1) * sin(t);
AY = y + (params.L_trailer_front_hang + buffer_1) * sin(t) + (params.L_half_width + buffer_1) * cos(t);
BX = x + (params.L_trailer_front_hang + buffer_1) * cos(t) + (params.L_half_width + buffer_1) * sin(t);
BY = y + (params.L_trailer_front_hang + buffer_1) * sin(t) - (params.L_half_width + buffer_1) * cos(t);
CX = x - (params.L_trailer_rear_hang + buffer_1) * cos(t) + (params.L_half_width + buffer_1) * sin(t);
CY = y - (params.L_trailer_rear_hang + buffer_1) * sin(t) - (params.L_half_width + buffer_1) * cos(t);
DX = x - (params.L_trailer_rear_hang + buffer_1) * cos(t) - (params.L_half_width + buffer_1) * sin(t);
DY = y - (params.L_trailer_rear_hang + buffer_1) * sin(t) + (params.L_half_width + buffer_1) * cos(t);

for obs_id = 1 : params.Nobs
    for obs_vertex_id = 1 : 4
        obs_x = params.obs{obs_id}.x(obs_vertex_id);
        obs_y = params.obs{obs_id}.y(obs_vertex_id);
        dx=obs_x-x; dy=obs_y-y;
        local_x=dx.*cos(t)+dy.*sin(t);
        local_y=-dx.*sin(t)+dy.*cos(t);
        is_obs_vertex_in_vehicle_polygon=any(abs(local_x)<=1.05 & abs(local_y)<=1.05);
        params.mat_ov(obs_id, obs_vertex_id, jj, ii) = is_obs_vertex_in_vehicle_polygon;
    end
    
    if (any(inpolygon(AX, AY, params.obs{obs_id}.x, params.obs{obs_id}.y)))
        params.mat_vv(obs_id, jj, ii, 1) = 1;
    end
    if (any(inpolygon(BX, BY, params.obs{obs_id}.x, params.obs{obs_id}.y)))
        params.mat_vv(obs_id, jj, ii, 2) = 1;
    end
    if (any(inpolygon(CX, CY, params.obs{obs_id}.x, params.obs{obs_id}.y)))
        params.mat_vv(obs_id, jj, ii, 3) = 1;
    end
    if (any(inpolygon(DX, DY, params.obs{obs_id}.x, params.obs{obs_id}.y)))
        params.mat_vv(obs_id, jj, ii, 4) = 1;
    end
end
end