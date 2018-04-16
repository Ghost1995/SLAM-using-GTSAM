%% Wrapper for HW1 for CMSC828T Course at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
ToolboxPath = 'gtsam_toolbox';
addpath(ToolboxPath);

%% Load Data
load('HW1.mat');

%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(Odom, ObservedLandMarks, StartingPose);
% 
% error1 = AllPoseIdeal - AllPosesComputed;
% err1 = 0;
% for i=1:5
%     err1 = err1+norm(error1(:,i));
% end
% 
% err2 = norm(AllPoseIdeal(:,1)-StartingPose);
% for i=1:4
%     err2 = err2+norm(AllPoseIdeal(:,i+1)-StartingPose-sum(Odom(:,1:i),2));
% end
