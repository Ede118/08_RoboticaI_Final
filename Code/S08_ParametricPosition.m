% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/


%% Path: Curva paramétrica Cilíndrica 
clear; clc; close all;

% Parámetros Especiales
R = 0.6;                % Radio del cilindro [m]
A = 0.01;               % Amplitud de la oscilacion (tangencial) [m]
A_ang = A / R;          % Amplitud equivalente en [rad]
k = 20 * 2 * pi;        % Frecuencia: 20 ciclos exactos [rad]

% Puntos base
z1 = 0.2; z2 = 0.8;
theta1 = -pi/6; theta2 = pi/6;

pasos = 1001;

% Ley Temporal: Perfil Trapezoidal
[u, ud, udd] = lspb(0, 1, pasos); 

%% Generación de los 4 tramos (En coordenadas Cilíndricas)

% --- TRAMO 1: Subida (Z avanza, Theta oscila) ---
theta_T1 = theta1 + A_ang .* sin(k .* u);
Z_T1     = z1 + (z2 - z1) .* u;
X_T1     = R .* cos(theta_T1);
Y_T1     = R .* sin(theta_T1);

% --- TRAMO 2: Arco superior (Theta avanza, Z oscila) ---
theta_T2 = theta1 + (theta2 - theta1) .* u;
Z_T2     = z2 + A .* sin(k .* u);
X_T2     = R .* cos(theta_T2);
Y_T2     = R .* sin(theta_T2);

% --- TRAMO 3: Bajada (Z retrocede, Theta oscila) ---
theta_T3 = theta2 + A_ang .* sin(k .* u);
Z_T3     = z2 + (z1 - z2) .* u;
X_T3     = R .* cos(theta_T3);
Y_T3     = R .* sin(theta_T3);

% --- TRAMO 4: Arco inferior (Theta retrocede, Z oscila) ---
theta_T4 = theta2 + (theta1 - theta2) .* u;
Z_T4     = z1 + A .* sin(k .* u);
X_T4     = R .* cos(theta_T4);
Y_T4     = R .* sin(theta_T4);


%% Concatenación de la trayectoria completa
Trayectoria_X = [X_T1; X_T2; X_T3; X_T4];
Trayectoria_Y = [Y_T1; Y_T2; Y_T3; Y_T4];
Trayectoria_Z = [Z_T1; Z_T2; Z_T3; Z_T4];

Posiciones_Cartesianas = [Trayectoria_X, Trayectoria_Y, Trayectoria_Z];

%% Visualización 3D
figure('Color','w', 'Name', 'Planificación de Soldadura Cilíndrica');

% Dibujar el "cilindro"
[Xc, Yc, Zc] = cylinder(R, 50);
Zc = Zc * (z2 + 0.2);
surf(Xc, Yc, Zc, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;

% Dibujar la trayectoria continua
plot3(Trayectoria_X, Trayectoria_Y, Trayectoria_Z, 'r-', 'LineWidth', 2);
plot3(Trayectoria_X(1), Trayectoria_Y(1), Trayectoria_Z(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); 

grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Trayectoria de Soldadura Cilíndrica 3D Continua');
legend('Pieza (Cilindro)', 'Trayectoria del TCP', 'Punto de Inicio');
view(45, 20);
