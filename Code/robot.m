% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

clear; clc; close all

%% Definicion de los parametros del robot

% DH = [
%     0       0.450   0.075    pi/2   0;   % Joint 1
%     0       0.000   0.640    0      0;   % Joint 2
%     0       0.000   0.195    pi/2   0;   % Joint 3
%     0       0.700   0.000    -pi/2  0;   % Joint 4
%     0       0.000   0.000    -pi/2  0;   % Joint 5
%     0       0.075   0.000    0      0];  % Joint 6
% %  theta      d       a     alpha  sigma

DH = [
    0       0.450   0.075    pi/2   0;   % Joint 1
    0       0.000   0.640    0      0;   % Joint 2
    0       0.000   0.195    -pi/2   0;   % Joint 3
    0       -0.700   0.000   pi/2  0;   % Joint 4
    0       0.000   0.000    -pi/2  0;   % Joint 5
    0       0.075   0.000    -pi      0];  % Joint 6
%  theta      d       a     alpha  sigma


name = 'ARC Mate 100iD';
qlim = deg2rad([ ...
   -170  170;       % q1
    80  -140;       % q2
   -170  170;       % q3
   -180  180;       % q4
   -120  120;       % q5
   -360  360]);     % q6


offset = deg2rad([
    90; 
    75; 
    25; 
    0; 
    180; 
    0]);

base = transl(0,0,0);
d_tool = 0.016; 
tool = transl(d_tool,0,0);

%% Definicion del robot
R = SerialLink(DH);
R.name = name;
R.qlim = qlim;
R.offset = offset;
R.base = base;
R.tool = tool;