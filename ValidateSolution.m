function report=ValidateSolution(r)
% Independent checks of the published discrete NLP solution; no swept-volume test.
p=r.params; dt=r.tf/p.NFE;
residual=[diff(r.x(:,1))-dt*r.v(1:end-1,1).*cos(r.theta(1:end-1,1)); ...
    diff(r.y(:,1))-dt*r.v(1:end-1,1).*sin(r.theta(1:end-1,1)); ...
    diff(r.theta(:,1))-dt*r.v(1:end-1,1).*tan(r.phy(1:end-1))/1.5; ...
    diff(r.v(:,1))-dt*r.a(1:end-1); diff(r.phy)-dt*r.w(1:end-1)];
for body=2:4
    delta=r.theta(:,body-1)-r.theta(:,body);
    residual=[residual; r.x(:,body)-r.x(:,body-1)+3*cos(r.theta(:,body)); ...
        r.y(:,body)-r.y(:,body-1)+3*sin(r.theta(:,body)); ...
        3*diff(r.theta(:,body))-dt*r.v(1:end-1,body-1).*sin(delta(1:end-1)); ...
        r.v(:,body)-r.v(:,body-1).*cos(delta)]; %#ok<AGROW>
end
boundary=[];
for body=1:4
    boundary=[boundary;r.x(1,body)-p.(sprintf('x%d_init',body));r.y(1,body)-p.(sprintf('y%d_init',body)); ...
        r.theta(1,body)-p.(sprintf('theta%d_init',body));r.x(end,body)-p.(sprintf('x%d_end_norm',body)); ...
        r.y(end,body)-p.(sprintf('y%d_end_norm',body));r.theta(end,body)-p.(sprintf('theta%d_end_norm',body)); ...
        r.v(1,body);r.v(end,body)]; %#ok<AGROW>
end
boundary=[boundary;r.a([1,end]);r.phy([1,end]);r.w([1,end])];
report.dynamics_residual=max(abs(residual)); report.boundary_residual=max(abs(boundary));
report.bound_violation=max([0;abs(r.v(:,1))-2.5;abs(r.a)-.25;abs(r.phy)-.7;abs(r.w)-.5; ...
    reshape(abs(diff(r.theta,1,2))-(pi/2-.1),[],1)]);
report.collision_pairs=0; report.self_collision_pairs=0; report.workspace_violation=0;
for i=1:p.nfe
    polygons=cell(1,4);
    for body=1:4
        q=VehiclePolygon(r.x(i,body),r.y(i,body),r.theta(i,body),body); polygons{body}=q;
        report.workspace_violation=max(report.workspace_violation,max(abs(q(:)))-25);
        for obstacle=1:p.Nobs
            o=p.obs{obstacle}; polygon=[o.x(1:end-1)',o.y(1:end-1)'];
            if PolygonsOverlap(q,polygon), report.collision_pairs=report.collision_pairs+1; end
        end
    end
    for a=1:3
        for b=a+1:4
            if PolygonsOverlap(polygons{a},polygons{b}), report.self_collision_pairs=report.self_collision_pairs+1; end
        end
    end
end
report.finite=all(isfinite([r.x(:);r.y(:);r.theta(:);r.v(:);r.a;r.w;r.phy;r.tf]));
report.passed=report.finite && report.dynamics_residual<1e-5 && report.boundary_residual<1e-5 && ...
    report.bound_violation<1e-5 && report.workspace_violation<1e-5 && ...
    report.collision_pairs==0 && report.self_collision_pairs==0;
end

function overlap=PolygonsOverlap(a,b)
% Separating axes catch edge crossings as well as contained vertices.
overlap=true;
for polygon={a,b}
    q=polygon{1}; edges=q([2:end,1],:)-q;
    for i=1:size(edges,1)
        axis=[-edges(i,2);edges(i,1)]; axis=axis/max(norm(axis),eps);
        pa=a*axis; pb=b*axis;
        if max(pa)<=min(pb)+1e-8 || max(pb)<=min(pa)+1e-8, overlap=false; return; end
    end
end
end
