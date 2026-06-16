% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

clc; close all;


%% Definicion de los parametros del robot

% Esta tiene notacion "mas comoda"
% Es la que esta en el informe
DH = [
    0       0.450   0.075    pi/2   0;   % Joint 1
    0       0.000   0.640    0      0;   % Joint 2
    0       0.000   0.195    pi/2   0;   % Joint 3
    0       0.700   0.000    -pi/2  0;   % Joint 4
    0       0.000   0.000    pi/2  0;   % Joint 5
    0       0.075   0.000    0      0];  % Joint 6
% %  theta      d       a     alpha  sigma

% Esta matriz me permite ver mejor la ETIQUETA DEL
% EJE q4. Tiene utilidad meramente estetica.
% DH = [
%     0       0.450   0.075    pi/2       0;   % Joint 1
%     0       0.000   0.640    0          0;   % Joint 2
%     0       0.000   0.195    -pi/2      0;   % Joint 3
%     0       -0.700   0.000   pi/2       0;   % Joint 4
%     0       0.000   0.000    -pi/2      0;   % Joint 5
%     0       0.075   0.000    -pi        0];  % Joint 6
% %  theta      d       a     alpha  sigma

name = 'ARC Mate 100iD';
qlim = deg2rad([ ...
   -170     170;       % qlim1
   -117.5   117.5;     % qlim2
   -170     170;       % qlim3
   -190     190;       % qlim4
   -180     180;       % qlim5
   -450     450]);     % qlim6


offset = deg2rad([
    90; 
    75; 
    25; 
    0; 
    0; 
    0]);

base = transl(0,0,0);
d_tool = 0; 
tool = transl(0,0,d_tool);

%% Definicion del robot

Robot = SerialLink(DH);
Robot.name = name;
Robot.qlim = qlim;
Robot.offset = offset;
Robot.base = base;
Robot.tool = tool;

%% Limites de WorkSpace
x1lim = -2;
x2lim = 2;
y1lim = -2;
y2lim = 2;
z1lim = -0.1;
z2lim = 2;

WS = [x1lim x2lim y1lim y2lim z1lim z2lim];