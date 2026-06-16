% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/


%% Path Completo: Posición y Orientación (Soldadura con Weaving e Inclinación)
clear; clc; close all;

% Parámetros Especiales
R = 1.2;                    % Radio del cilindro [m]
A = 0.01;                   % Amplitud del weaving [m]
A_ang = A / R;              % Amplitud en radianes
k = 20 * 2 * pi;            % Frecuencia de oscilación
alpha_tilt = deg2rad(15);   % Ángulo de avance (15°)

% Puntos base
z1 = 0.4; z2 = 1.0;
theta1 = -pi/6; theta2 = pi/6;
pasos = 1001;

% Ley Temporal
[u, ud, udd] = lspb(0, 1, pasos); 

%% Generación de los 4 tramos (Posición y Vector de Avance)

% --- TRAMO 1: Subida ---
theta_T1 = theta1 + A_ang .* sin(k .* u);
Z_T1     = z1 + (z2 - z1) .* u;
X_T1     = R .* cos(theta_T1);
Y_T1     = R .* sin(theta_T1);
% Dirección macro de avance: hacia arriba (+Z)
t_adv_T1 = repmat([0, 0, 1], pasos, 1); 

% --- TRAMO 2: Arco superior ---
theta_T2 = theta1 + (theta2 - theta1) .* u;
Z_T2     = z2 + A .* sin(k .* u);
X_T2     = R .* cos(theta_T2);
Y_T2     = R .* sin(theta_T2);
% Dirección macro de avance: tangencial horaria [-sin(th), cos(th), 0]
t_adv_T2 = [-sin(theta_T2), cos(theta_T2), zeros(pasos, 1)];

% --- TRAMO 3: Bajada ---
theta_T3 = theta2 + A_ang .* sin(k .* u);
Z_T3     = z2 + (z1 - z2) .* u;
X_T3     = R .* cos(theta_T3);
Y_T3     = R .* sin(theta_T3);
% Dirección macro de avance: hacia abajo (-Z)
t_adv_T3 = repmat([0, 0, -1], pasos, 1);

% --- TRAMO 4: Arco inferior ---
theta_T4 = theta2 + (theta1 - theta2) .* u;
Z_T4     = z1 + A .* sin(k .* u);
X_T4     = R .* cos(theta_T4);
Y_T4     = R .* sin(theta_T4);
% Dirección macro de avance: tangencial antihoraria
t_adv_T4 = [sin(theta_T4), -cos(theta_T4), zeros(pasos, 1)];


%% Concatenación de datos espaciales
Trayectoria_X = [X_T1; X_T2; X_T3; X_T4];
Trayectoria_Y = [Y_T1; Y_T2; Y_T3; Y_T4];
Trayectoria_Z = [Z_T1; Z_T2; Z_T3; Z_T4];
Trayectoria_Theta = [theta_T1; theta_T2; theta_T3; theta_T4];
Trayectoria_Adv   = [t_adv_T1; t_adv_T2; t_adv_T3; t_adv_T4];

Total_Pasos = length(Trayectoria_X);

%% ZONA DE EMPALME / BLENDING
% Definimos cuántos puntos usará el robot para "redondear" la transición.
% Se define el "porcentaje de empalme".

porcentaje_empalme = 0.02;
zona_empalme = round(pasos*porcentaje_empalme); 

% 1. Suavizamos la Geometría
Trayectoria_X = smoothdata(Trayectoria_X, 'gaussian', zona_empalme);
Trayectoria_Y = smoothdata(Trayectoria_Y, 'gaussian', zona_empalme);
Trayectoria_Z = smoothdata(Trayectoria_Z, 'gaussian', zona_empalme);

% 2. Suavizamos la Orientación
Trayectoria_Adv = smoothdata(Trayectoria_Adv, 1, 'gaussian', zona_empalme);

% Se vuelven a normalizar los vectores para que las matemáticas de la rotación no fallen.
Trayectoria_Adv = Trayectoria_Adv ./ vecnorm(Trayectoria_Adv, 2, 2);

%% Cálculo de la Orientación Dinámica [Push Angle]
CPosition = zeros(Total_Pasos, 6); 

for i = 1:Total_Pasos
    th = Trayectoria_Theta(i);
    t_adv = Trayectoria_Adv(i, :);
    
    % a) Vector de Aproximación (Z de la antorcha) = Normal Interior
    a_vec = [cos(th); sin(th); 0];
    
    % b) Construir Base de Rotación Perpendicular Exacta [n, o, a]
    % Usar el eje Z del mundo para asegurar ortogonalidad en el plano coordenado
    z_mundo = [0; 0; 1];
    n_vec = cross(z_mundo, a_vec); 
    n_vec = n_vec / norm(n_vec);
    o_vec = cross(a_vec, n_vec); 
    
    R_perp = [n_vec, o_vec, a_vec]; % Matriz perpendicular perfecta
    
    % c) Aplicar la inclinación de 15° en la dirección de avance
    % El eje de giro óptimo es perpendicular a la normal (a) y al avance (t_adv)
    eje_giro = cross(a_vec, t_adv');
    
    if norm(eje_giro) > 1e-6
        eje_giro = eje_giro / norm(eje_giro);
        % angvec2r genera la matriz de rotación pura a partir de un eje y un ángulo
        R_tilt = angvec2r(alpha_tilt, eje_giro); 
        R_final = R_tilt * R_perp;
    else
        R_final = R_perp; 
    end
    
    % d) Convertir la matriz 3x3 a ángulos de Euler Roll-Pitch-Yaw
    rpy = tr2rpy(R_final); % Devuelve [roll, pitch, yaw] en radianes
    
    % e) Guardar en el formato (6 x K)
    CPosition(i, :) = [Trayectoria_X(i); Trayectoria_Y(i); Trayectoria_Z(i); rpy'];
end

%% Cinemática Inversa con Semilla Óptima (Front - Elbow Up)
S01_my_robot;

% q1 apunta al inicio (theta1), q2 inclina hombro adelante, q3 levanta codo
q_semilla_inicial = [theta1, pi/4, -pi/4, 0, pi/2, 0]; 

[Q] = CinematicaInversa(Robot, CPosition, q_semilla_inicial);

fprintf('Matriz Target_Poses (%dx6) generada con éxito.\n', size(Q, 1));

%% Análisis Cinemático en el Espacio Cartesiano - Por Tramo

% 1. Definir el tiempo físico de la trayectoria
tiempo_por_tramo = 5; 
t = linspace(0, tiempo_por_tramo, pasos);
dt = t(2) - t(1); 

% 2. Agrupar las coordenadas
Tramos_X = {X_T1, X_T2, X_T3, X_T4};
Tramos_Y = {Y_T1, Y_T2, Y_T3, Y_T4};
Tramos_Z = {Z_T1, Z_T2, Z_T3, Z_T4};
Nombres = {'Tramo 1 (Subida)', 'Tramo 2 (Arco Sup)', 'Tramo 3 (Bajada)', 'Tramo 4 (Arco Inf)'};

% 3. Bucle de cálculo y graficación
carpeta_destino_C = 'Graficos_Cinematica_Cartesiana_IN';
if ~exist(carpeta_destino_C, 'dir')
    mkdir(carpeta_destino_C);
end

for i = 1:4
    % Extraer posiciones del tramo actual
    X = Tramos_X{i};
    Y = Tramos_Y{i};
    Z = Tramos_Z{i};

    % Cálculo Numérico de Velocidades (dx/dt, dy/dt, dz/dt)
    Vx = gradient(X, dt);
    Vy = gradient(Y, dt);
    Vz = gradient(Z, dt);

    % Cálculo Numérico de Aceleraciones (dv/dt)
    Ax = gradient(Vx, dt);
    Ay = gradient(Vy, dt);
    Az = gradient(Vz, dt);

    % --- CREACIÓN DE LA FIGURA ---
    fig1 = figure('Color', 'w', 'Name', ['Posicion - ' Nombres{i}]);
    plot(t, [X Y Z], 'LineWidth', 1.5);
    title(['Posición Cartesiana - ' Nombres{i}]);
    ylabel('Posición [m]'); xlabel('Tiempo [s]');
    lgdX = legend('X', 'Y', 'Z', 'Location', 'eastoutside'); lgdX.ItemHitFcn = @toggleSignal;
    grid on; grid minor;

    nombre_pos = fullfile(carpeta_destino_C, sprintf('Tramo_%d_Posicion.png', i));
    exportgraphics(fig1, nombre_pos, 'Resolution', 300);

    % Gráfico de Velocidad
    fig2 = figure('Color', 'w', 'Name', ['Velocidad - ' Nombres{i}]);
    plot(t, [Vx Vy Vz], 'LineWidth', 1.5);
    title(['Velocidad Cartesiana - ' Nombres{i}]);
    ylabel('Velocidad [m/s]'); xlabel('Tiempo [s]');
    lgdV = legend('V_x', 'V_y', 'V_z', 'Location', 'eastoutside'); lgdV.ItemHitFcn = @toggleSignal;
    grid on; grid minor;

    nombre_vel = fullfile(carpeta_destino_C, sprintf('Tramo_%d_Velocidad.png', i));
    exportgraphics(fig2, nombre_vel, 'Resolution', 300);

    % Gráfico de Aceleración
    fig3 = figure('Color', 'w', 'Name', ['Aceleración - ' Nombres{i}]);
    plot(t, [Ax Ay Az], 'LineWidth', 1.5);
    title(['Aceleración Cartesiana - ' Nombres{i}]);
    ylabel('Acel. [m/s^2]'); xlabel('Tiempo [s]');
    lgdA = legend('A_x', 'A_y', 'A_z', 'Location', 'eastoutside'); lgdA.ItemHitFcn = @toggleSignal;
    grid on; grid minor;

    nombre_acc = fullfile(carpeta_destino_C, sprintf('Tramo_%d_Aceleracion.png', i));
    exportgraphics(fig3, nombre_acc, 'Resolution', 300);
end

disp('Los 12 gráficos han sido guardados en la carpeta "Graficos_Cinematica".');

%% Análisis Cinemático en el Espacio Articular (Motores)

% 1. Definir el tiempo físico total
cant_tramos = 4;
tiempo_total = cant_tramos*tiempo_por_tramo; 
t_total = linspace(0, tiempo_total, Total_Pasos);
dt_total = t_total(2) - t_total(1);

% 2. Inicializar matrices para Velocidad y Aceleración Articular
V_art = zeros(Total_Pasos, 6);
A_art = zeros(Total_Pasos, 6);

% 3. Cálculo Numérico usando 'gradient' para cada una de las 6 articulaciones

carpeta_destino_Q = 'Graficos_Cinematica_Articular_IN';
if ~exist(carpeta_destino_Q, 'dir')
    mkdir(carpeta_destino_Q);
end

for j = 1:6
    V_art(:, j) = gradient(Q(:, j), dt_total);
    A_art(:, j) = gradient(V_art(:, j), dt_total);
end

nombres_ejes = {'q_1 (Base)', 'q_2 (Hombro)', 'q_3 (Codo)', 'q_4 (Muñeca 1)', 'q_5 (Muñeca 2)', 'q_6 (Muñeca 3)'};

% --- GRÁFICO 1: POSICIÓN ARTICULAR ---
fig_q = figure('Color', 'w', 'Name', 'Posición Articular');
% Multiplicamos por 180/pi si prefieres ver los ángulos en GRADOS (opcional)
plot(t_total, rad2deg(Q), 'LineWidth', 1.5); 
title('Evolución de la Posición Articular');
ylabel('Posición [deg]'); xlabel('Tiempo [s]');
lgdQ = legend(nombres_ejes, 'Location', 'eastoutside'); lgdQ.ItemHitFcn = @toggleSignal;
grid on; grid minor;

% Guardar imagen
nombre_q = fullfile(carpeta_destino_Q, 'Articular_1_Posicion.png');
exportgraphics(fig_q, nombre_q, 'Resolution', 300);

% --- GRÁFICO 2: VELOCIDAD ARTICULAR ---
fig_vq = figure('Color', 'w', 'Name', 'Velocidad Articular');
plot(t_total, rad2deg(V_art), 'LineWidth', 1.5);
title('Evolución de la Velocidad Articular');
ylabel('Velocidad [deg/s]'); xlabel('Tiempo [s]');
lgdQd = legend(nombres_ejes, 'Location', 'eastoutside'); lgdQd.ItemHitFcn = @toggleSignal;
grid on; grid minor;

nombre_vq = fullfile(carpeta_destino_Q, 'Articular_2_Velocidad.png');
exportgraphics(fig_vq, nombre_vq, 'Resolution', 300);

% --- GRÁFICO 3: ACELERACIÓN ARTICULAR ---
fig_aq = figure('Color', 'w', 'Name', 'Aceleración Articular');
plot(t_total, rad2deg(A_art), 'LineWidth', 1.5);
title('Evolución de la Aceleración Articular');
ylabel('Aceleración [deg/s^2]'); xlabel('Tiempo [s]');
lgdQdd = legend(nombres_ejes, 'Location', 'eastoutside'); lgdQdd.ItemHitFcn = @toggleSignal;
grid on; grid minor;

% Guardar imagen
nombre_aq = fullfile(carpeta_destino_Q, 'Articular_3_Aceleracion.png');
exportgraphics(fig_aq, nombre_aq, 'Resolution', 300);

disp('Los 3 gráficos articulares han sido guardados.');

pause()


%% Configuración del Gráfico de Simulación
x1lim = -2; x2lim = 2;
y1lim = -2; y2lim = 2;
z1lim = -0.1; z2lim = 2;
WS = [x1lim x2lim y1lim y2lim z1lim z2lim];

figure('Color','w', 'Name', 'Simulación de Soldadura Interna'); grid on; 
hold on;

% 1. Dibujar el cilindro de la pieza
[Xc, Yc, Zc] = cylinder(R, 50);
Zc = Zc * (z2 + 0.2); 
surf(Xc, Yc, Zc, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.4);

% 2. Trayectoria cartesiana directamente sobre la pieza
% Esto actúa como el cordón de soldadura ya trazado perfectamente en el espacio
plot3(Trayectoria_X, Trayectoria_Y, Trayectoria_Z, 'r-', 'LineWidth', 1);
plot3(Trayectoria_X(1), Trayectoria_Y(1), Trayectoria_Z(1), 'g.', 'MarkerSize', 20); % Inicio en verde

% 3. Graficar la base del robot estática para congelar los ejes y aplicar axis equal
Robot.plot(zeros(1, Robot.n), 'workspace', WS, 'notiles', 'scale', 0.4, 'jointdiam', 0.8);
axis equal;
view(135, 25);

% 4. Iniciar la animación del brazo robótico recorriendo el cordón
disp('Animando trayectoria en configuración Front-Elbow Up...');
grabar = false;

if grabar
    Robot.plot(Q, 'workspace', WS, 'notiles', 'scale', 0.4, 'jointdiam', 0.8, 'fps', 60, 'movie', 'Simulacion_Soldadura_Outside.mp4'); % , 'movie', 'Simulacion_Soldadura.mp4'
else
    Robot.plot(Q, 'workspace', WS, 'notiles', 'scale', 0.4, 'jointdiam', 0.8, 'fps', 60);
end

disp('End of Animation')
