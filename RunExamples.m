% Recompute both examples from scratch, exporting two static figures per case.
clear; close all; clc;
results = cell(1,2);
for case_id = 1:2
    close all;
    results{case_id} = PlanTrajectory(case_id);
    PlotResults(results{case_id});
end
