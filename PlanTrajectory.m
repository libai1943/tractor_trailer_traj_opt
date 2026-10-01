function result = PlanTrajectory(case_id)
% Paper Algorithm 3: planar A* -> LIOS -> trust-region maneuver optimization.
global params
root=fileparts(mfilename('fullpath')); addpath(root);
previous=pwd; restore=onCleanup(@()cd(previous));
folder=fullfile(root,'RunData',sprintf('case_%d',case_id));
if ~isfolder(folder), mkdir(folder); end
cd(folder);
LoadCase(case_id); InitializeParams();
assert(all(params.M==0),'These two examples implement the standard three-trailer vehicle (zero hitch offsets).');
timer=tic;
fprintf('Case %d | Stage 1: planar A* guiding paths.\n',case_id);
paths=SearchGuidingPath();
guess=WriteInitialGuess(paths);
history=struct('stage',{},'iteration',{},'duration',{},'infeasibility',{});
fprintf('Stage 2: LIOS feasibility recovery.\n');
for iter=1:params.max_iter
    reference=ToCenterPaths(guess);
    SpecifyLocalBoxes(reference);
    WriteSolutionGuess(guess,2);
    guess=SolveNLP(root,2,iter);
    history(end+1)=struct('stage',2,'iteration',iter,'duration',guess.tf,'infeasibility',guess.infeasibility); %#ok<AGROW>
    fprintf('  LIOS %d: T=%.6f s, zeta=%.6g.\n',iter,guess.tf,guess.infeasibility);
    if guess.infeasibility<params.feasibility_tolerance, break; end
end
feasible_guess=guess;
fprintf('Stage 3: trust-region maneuver optimization.\n');
for iter=1:params.max_iter_trmo
    previous_time=guess.tf;
    UpdateNlpFormulation();
    WriteSolutionGuess(guess,3);
    guess=SolveNLP(root,3,iter);
    history(end+1)=struct('stage',3,'iteration',iter,'duration',guess.tf,'infeasibility',0); %#ok<AGROW>
    fprintf('  TRMO %d: T=%.6f s, decrease=%.6f s.\n',iter,guess.tf,previous_time-guess.tf);
    if previous_time-guess.tf<params.cost_tolerance, break; end
end
result=guess; result.case_id=case_id; result.params=params;
result.guiding_paths=paths; result.feasibility_result=feasible_guess;
result.history=history; result.wall_seconds=toc(timer);
result.validation=ValidateSolution(result);
assert(result.validation.passed,'Final trajectory failed independent validation; see result metrics.');
save(fullfile(folder,'result.mat'),'result');
report=struct('case_id',case_id,'duration',result.tf,'wall_seconds',result.wall_seconds, ...
    'finite_elements',params.NFE,'configuration_points',params.nfe,'history',history,'validation',result.validation);
fid=fopen(fullfile(folder,'validation.json'),'w'); fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true)); fclose(fid);
fprintf('Validated: T=%.6f s, dynamics residual %.3g, collision pairs %d.\n', ...
    result.tf,result.validation.dynamics_residual,result.validation.collision_pairs);
end

function paths=ToCenterPaths(r)
for body=1:4
    x=r.x(:,body); y=r.y(:,body);
    if body==1, x=x+.75*cos(r.theta(:,body)); y=y+.75*sin(r.theta(:,body)); end
    paths.(sprintf('path%d',body))=struct('x',x','y',y');
end
end
