% Chapter 9 companion code for the Chinese-language book
% 《非结构化场景自动驾驶轨迹规划技术》.
% English title translation: Trajectory Planning Techniques for Autonomous
% Driving in Unstructured Environments. The book is written in Chinese.
%
% Please cite Bai Li, Li Li, Tankut Acarman, Zhijiang Shao, and Ming Yue,
% "Optimization-Based Maneuver Planning for a Tractor-Trailer Vehicle in a
% Curvy Tunnel: A Weak Reliance on Sampling and Search," IEEE Robotics and
% Automation Letters, 7(2):706-713, 2022. DOI:10.1109/LRA.2021.3131693.
%
% Set case_id to 1 or 2, then click Run. Each case produces two static figures.
% Windows MATLAB + Image Processing Toolbox; local AMPL/IPOPT executables.
% A* is distributed as SearchGuidingPath.p. No video implementation is supplied.
% Noncommercial use: see LICENSE and README.md.
clear; close all; clc;
case_id = 1; % 1: U-shaped tunnel; 2: cluttered curvy tunnel (paper Fig. 4).
result = PlanTrajectory(case_id);
PlotResults(result);
