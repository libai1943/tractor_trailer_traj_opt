function WriteSolutionGuess(r,stage)
% Fresh solver initialization; stage 3 corner geometry is computed directly.
fid=fopen('ig.INIVAL','w'); cleanup=onCleanup(@()fclose(fid));
for body=1:4
    names={'x','y','theta','v'};
    for k=1:numel(names)
        values=r.(names{k})(:,body);
        for i=1:numel(values), fprintf(fid,'let %s[%d,%d] := %.17g;\n',names{k},i,body,values(i)); end
    end
end
for name={'a','phy','w'}
    values=r.(name{1});
    for i=1:numel(values), fprintf(fid,'let %s[%d] := %.17g;\n',name{1},i,values(i)); end
end
if stage==2
    for i=1:size(r.x,1)
        fprintf(fid,'let xc[%d] := %.17g;\n',i,r.x(i,1)+.75*cos(r.theta(i,1)));
        fprintf(fid,'let yc[%d] := %.17g;\n',i,r.y(i,1)+.75*sin(r.theta(i,1)));
    end
else
    for body=1:4
        for i=1:size(r.x,1)
            polygon=VehiclePolygon(r.x(i,body),r.y(i,body),r.theta(i,body),body);
            names={'A','B','C','D'};
            for vertex=1:4
                fprintf(fid,'let %sX[%d,%d] := %.17g;\n',names{vertex},i,body,polygon(vertex,1));
                fprintf(fid,'let %sY[%d,%d] := %.17g;\n',names{vertex},i,body,polygon(vertex,2));
            end
        end
    end
end
fprintf(fid,'let tf := %.17g;\n',r.tf);
end
