%MDL_Sorter Create model of Sorter manipulator
%
%      mdl_sorter
%
% Script creates the workspace variable Sorter which describes the 
% kinematic characterstics of a Sorter 6dof manipulator using
% modified DH conventions.
%
% Also defines the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%
%
% This file is conformed to The Robotics Toolbox for Matlab.
% 

% IRp-6 Kinematics
   d1 = 0.70; % base height
   a2 = 0.45; % vertical link length
   a3 = 0.67; % horizontal link length
   d5 = 0.15; % 6. DOF displacement
   d7 = 0.20; % gripper length

clear L
%            theta    d        a    alpha
L(1) = Link([  0      0       0      0     0], 'modified');
L(2) = Link([  0      0        0      -pi/2 0], 'modified');
L(3) = Link([  0      0        a2     0     0], 'modified');
L(4) = Link([  0      0        a3     0     0], 'modified');
L(5) = Link([  0      d5       0      -pi/2 0], 'modified');
L(6) = Link([  0      0        0      pi/2  0], 'modified');

qz = [0 0 0 0 0 0]; % zero angles\
qr = [0 -pi/2 pi/2 -pi/2 -pi/2 0]; % ready pose, gripper points down

irp6_m = SerialLink(L, 'name', 'IRp6', 'comment', 'DH zmodyfikowane');
clear L

%irp6_m.base = [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
irp6_m.tool = [1 0 0 0; 0 1 0 0; 0 0 1 d7; 0 0 0 1];
