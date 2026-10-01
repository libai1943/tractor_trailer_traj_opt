function r=SolveNLP(root,stage,iteration)
% AMPL executable + run file + numeric TXT exchange; no MATLAB solver API.
models={'Feasibility.mod','Refinement.mod'}; model=models{stage-1};
copyfile(fullfile(root,model),model); copyfile(fullfile(root,'ipopt.opt'),'ipopt.opt');
ampl=getenv('AMPL_EXECUTABLE'); if isempty(ampl), ampl=fullfile(root,'ampl.exe'); end
ipopt=getenv('IPOPT_EXECUTABLE'); if isempty(ipopt), ipopt=fullfile(root,'ipopt.exe'); end
assert(isfile(ampl) && isfile(ipopt),'Install AMPL/IPOPT; see README.');
% A relative solver path avoids legacy AMPL's ANSI/UTF-8 path ambiguity.
solver_path=ipopt;
if strcmpi(ipopt,fullfile(root,'ipopt.exe')), solver_path='../../ipopt.exe'; end
files={'x','y','theta','v','a','phy','w','terminal_time','status','infeasibility'};
for k=1:numel(files), name=[files{k},'.txt']; if isfile(name), delete(name); end; end
fid=fopen('Solve.run','w');
fprintf(fid,'reset;\nmodel %s;\ninclude ig.INIVAL;\noption solver "%s";\nsolve;\n',model,strrep(solver_path,'\','/'));
fprintf(fid,'printf "%%d\\n", solve_result_num > "status.txt";\nclose "status.txt";\n');
fprintf(fid,'if solve_result_num >= 0 and solve_result_num < 100 then {\n');
for name={'x','y','theta','v'}
    fprintf(fid,'printf {car in 1..Nv, i in I} "%%.17g\\n", %s[i,car] > "%s.txt";\nclose "%s.txt";\n',name{1},name{1},name{1});
end
for name={'a','phy','w'}
    fprintf(fid,'printf {i in I} "%%.17g\\n", %s[i] > "%s.txt";\nclose "%s.txt";\n',name{1},name{1},name{1});
end
fprintf(fid,'printf "%%.17g\\n", tf > "terminal_time.txt";\nclose "terminal_time.txt";\n');
if stage==2, fprintf(fid,'printf "%%.17g\\n", (objective_-tf)/w_penalty > "infeasibility.txt";\nclose "infeasibility.txt";\n'); end
fprintf(fid,'}\n'); fclose(fid);
[exit_code,log]=system(sprintf('"%s" Solve.run 2>&1',ampl));
fid=fopen(sprintf('stage%d_%02d.log',stage,iteration),'w'); fprintf(fid,'%s',log); fclose(fid);
assert(exit_code==0 && isfile('status.txt'),'AMPL failed: inspect the stage log in RunData.');
status=readmatrix('status.txt');
assert(isscalar(status) && status>=0 && status<100,'IPOPT failed at stage %d iteration %d (status %g).',stage,iteration,status);
global params
for name={'x','y','theta','v'}
    values=readmatrix([name{1},'.txt']); assert(numel(values)==4*params.nfe && all(isfinite(values)));
    r.(name{1})=reshape(values,params.nfe,4);
end
for name={'a','phy','w'}
    r.(name{1})=readmatrix([name{1},'.txt']);
end
r.tf=readmatrix('terminal_time.txt'); r.t=linspace(0,r.tf,params.nfe)';
r.infeasibility=0; if stage==2, r.infeasibility=readmatrix('infeasibility.txt'); end
end
