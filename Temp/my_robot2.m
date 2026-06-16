% =========================================================================
% Simulación 3D - Proyecto Final Robótica I
% Robot: FANUC 10iD
% =========================================================================

clear; clc; close all;

%% 1. Ruta de los STLs (El argumento 'path')
% Aquí debes tener guardados tus archivos: link0.stl, link1.stl ... link6.stl
% ¡Asegúrate de que sean .stl y no .stp!
ruta_stls = fullfile('Graficos, Videos y STLs', 'FANUC 10iD');

%% 2. Definición Matemática (Pura, sin ensuciarla con CAD)
% Usamos tu cinemática tal cual
L(1) = Revolute('d', 0.450, 'a', 0.075, 'alpha',  pi/2);
L(2) = Revolute('d', 0.000, 'a', 0.640, 'alpha',     0);
L(3) = Revolute('d', 0.000, 'a', 0.195, 'alpha',  pi/2);
L(4) = Revolute('d', 0.700, 'a', 0.000, 'alpha', -pi/2);
L(5) = Revolute('d', 0.000, 'a', 0.000, 'alpha',  pi/2);
L(6) = Revolute('d', 0.075, 'a', 0.000, 'alpha',     0);

Robot = SerialLink(L, 'name', 'Fanuc_10iD');

%% 3. Entorno y Gráfico
WS = [-2 2 -2 2 -0.1 2]; 

figure('Color', 'w', 'Name', 'Simulación 3D - Proyecto Final');
view(135, 25); 
grid on; 

q_prueba = [0, 0, 0, 0, 0, 0];

disp('Generando el modelo 3D del robot...');

% ¡LA MAGIA OCURRE AQUÍ!
% Con el argumento 'path', el Toolbox busca, carga y ensambla 
% automáticamente los link0.stl a link6.stl
Robot.plot3d(q_prueba, 'workspace', WS, 'path', ruta_stls, 'color', [1 0.8 0]);

% Ajustamos la iluminación para que las piezas no se vean planas
camlight('headlight'); 
lighting gouraud;