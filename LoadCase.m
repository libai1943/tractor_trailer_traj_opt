function LoadCase(case_id)
% Two original environments, consolidated in one data file.
global params
assert(ismember(case_id,[1,2]),'case_id must be 1 or 2.');
root=fileparts(mfilename('fullpath'));
data=load(fullfile(root,'cases_data.mat'),'cases');
params=data.cases{case_id};
end
