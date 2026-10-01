function PlotResults(r)
% Open static visualization: vehicle footprints first, trajectory curves on top.
root=fileparts(mfilename('fullpath')); folder=fullfile(root,'docs','images');
if ~isfolder(folder), mkdir(folder); end
colors=[.84,.16,.18;.06,.46,.78;.12,.61,.33;.96,.53,.08];
names={'Tractor','Trailer 1','Trailer 2','Trailer 3'};
fig=figure('Color','w','Name',sprintf('Case %d | Trajectories',r.case_id),'Position',[80,70,900,840]);
ax=axes(fig); hold(ax,'on'); axis(ax,'equal'); box(ax,'on'); grid(ax,'on');
for k=1:r.params.Nobs
    o=r.params.obs{k}; patch(ax,o.x,o.y,[.46,.49,.52],'EdgeColor','none','HandleVisibility','off');
end
% Every optimized configuration is represented; no interpolation is used here.
for body=4:-1:1
    for i=1:size(r.x,1)
        q=VehiclePolygon(r.x(i,body),r.y(i,body),r.theta(i,body),body);
        patch(ax,q(:,1),q(:,2),colors(body,:),'FaceAlpha',.025, ...
            'EdgeColor',.50*colors(body,:)+.50,'LineWidth',.55,'HandleVisibility','off');
    end
end
handles=gobjects(1,4);
for body=1:4
    handles(body)=plot(ax,r.x(:,body),r.y(:,body),'Color',colors(body,:),'LineWidth',1.8);
    for i=[1,size(r.x,1)]
        q=VehiclePolygon(r.x(i,body),r.y(i,body),r.theta(i,body),body);
        patch(ax,q(:,1),q(:,2),colors(body,:),'FaceAlpha',.2,'EdgeColor',colors(body,:), ...
            'LineWidth',1.3,'HandleVisibility','off');
    end
end
plot(ax,r.x(1,1),r.y(1,1),'ko','MarkerFaceColor','w','HandleVisibility','off');
plot(ax,r.x(end,1),r.y(end,1),'kp','MarkerFaceColor','w','MarkerSize',12,'HandleVisibility','off');
legend(ax,handles,names,'Location','northoutside','Orientation','horizontal');
xlim(ax,[-25,25]); ylim(ax,[-25,25]); xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
title(ax,sprintf('Chapter 9 | Case %d | T = %.3f s',r.case_id,r.tf));
set(ax,'FontSize',12);
exportgraphics(fig,fullfile(folder,sprintf('case%d_trajectory.png',r.case_id)),'Resolution',150);

fig=figure('Color','w','Name',sprintf('Case %d | States and controls',r.case_id),'Position',[100,50,1200,830]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(layout); hold(ax,'on');
for body=1:4, plot(ax,r.t,r.v(:,body),'Color',colors(body,:),'LineWidth',1.4); end
yline(ax,2.5,'k:'); yline(ax,-2.5,'k:'); ylabel(ax,'v_i (m/s)'); legend(ax,names,'Location','best');
StyleAxes(ax,r.tf);
ax=nexttile(layout); hold(ax,'on');
for body=1:4, plot(ax,r.t,r.theta(:,body),'Color',colors(body,:),'LineWidth',1.4); end
ylabel(ax,'\theta_i (rad)'); StyleAxes(ax,r.tf);
ax=nexttile(layout); plot(ax,r.t,r.a,'Color',colors(1,:),'LineWidth',1.5); hold(ax,'on');
yline(ax,.25,'k:'); yline(ax,-.25,'k:'); ylabel(ax,'a_0 (m/s^2)'); StyleAxes(ax,r.tf);
ax=nexttile(layout); plot(ax,r.t,r.phy,'Color',colors(1,:),'LineWidth',1.5); hold(ax,'on');
yline(ax,.7,'k:'); yline(ax,-.7,'k:'); ylabel(ax,'\phi_0 (rad)'); StyleAxes(ax,r.tf);
ax=nexttile(layout); plot(ax,r.t,r.w,'Color',colors(1,:),'LineWidth',1.5); hold(ax,'on');
yline(ax,.5,'k:'); yline(ax,-.5,'k:'); ylabel(ax,'\omega_0 (rad/s)'); StyleAxes(ax,r.tf);
ax=nexttile(layout); hold(ax,'on');
for body=2:4, plot(ax,r.t,r.theta(:,body)-r.theta(:,body-1),'Color',colors(body,:),'LineWidth',1.4); end
yline(ax,pi/2-.1,'k:'); yline(ax,-pi/2+.1,'k:'); ylabel(ax,'Articulation angle (rad)'); StyleAxes(ax,r.tf);
title(layout,sprintf('Case %d | States and controls | 100 elements, 101 configurations',r.case_id));
exportgraphics(fig,fullfile(folder,sprintf('case%d_profiles.png',r.case_id)),'Resolution',150);
end

function StyleAxes(ax,duration)
grid(ax,'on'); box(ax,'on'); xlim(ax,[0,duration]); xlabel(ax,'Time (s)'); set(ax,'FontSize',11);
end
