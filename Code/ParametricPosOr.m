%% Path Completo: Posición y Orientación (Soldadura con Weaving e Inclinación)
clear; clc; close all;

% Parámetros Especiales
R = 1.2;                    % Radio del cilindro [m]
A = 0.01;                   % Amplitud del weaving [m]
A_ang = A / R;              % Amplitud en radianes
k = 20 * 2 * pi;            % Frecuencia de oscilación
alpha_tilt = deg2rad(15);   % Ángulo de avance (15°)

% Puntos base
z1 = 0.3; z2 = 1.0;
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

%% Cálculo de la Orientación Dinámica [Push Angle]
% Matriz de poses finales para la Cinemática Inversa (6 filas x K columnas)
CPosition = zeros(Total_Pasos, 6); 

for i = 1:Total_Pasos
    th = Trayectoria_Theta(i);
    t_adv = Trayectoria_Adv(i, :);
    
    % a) Vector de Aproximación (Z de la antorcha) = Normal Interior
    a_vec = [cos(th); sin(th); 0];
    
    % b) Construir Base de Rotación Perpendicular Exacta [n, o, a]
    % Usamos el eje Z del mundo para asegurar ortogonalidad en el plano coordenado
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

disp('¡Matriz Target_Poses (6x4004) generada con éxito!');

%% Análisis Cinemático (Posición, Velocidad y Aceleración) por Tramo

% 1. Definir el tiempo físico de la trayectoria
tiempo_por_tramo = 5; % Segundos que tarda el robot en hacer 1 tramo
t = linspace(0, tiempo_por_tramo, pasos);
dt = t(2) - t(1); % Diferencial de tiempo

% 2. Agrupar las coordenadas para facilitar el ploteo
Tramos_X = {X_T1, X_T2, X_T3, X_T4};
Tramos_Y = {Y_T1, Y_T2, Y_T3, Y_T4};
Tramos_Z = {Z_T1, Z_T2, Z_T3, Z_T4};
Nombres = {'Tramo 1 (Subida)', 'Tramo 2 (Arco Sup)', 'Tramo 3 (Bajada)', 'Tramo 4 (Arco Inf)'};

% 3. Bucle de cálculo y graficación
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
    figure('Color', 'w', 'Name', ['Cinemática - ' Nombres{i}]);

    % Gráfico de Posición
    subplot(3, 1, 1);
    plot(t, X, 'r', t, Y, 'g', t, Z, 'b', 'LineWidth', 1.5);
    title(['Posición Cartesiana - ' Nombres{i}]);
    ylabel('Posición [m]');
    legend('X', 'Y', 'Z', 'Location', 'eastoutside');
    grid on;

    % Gráfico de Velocidad
    subplot(3, 1, 2);
    plot(t, Vx, 'r', t, Vy, 'g', t, Vz, 'b', 'LineWidth', 1.5);
    title('Velocidad Cartesiana');
    ylabel('Velocidad [m/s]');
    legend('V_x', 'V_y', 'V_z', 'Location', 'eastoutside');
    grid on;

    % Gráfico de Aceleración
    subplot(3, 1, 3);
    plot(t, Ax, 'r', t, Ay, 'g', t, Az, 'b', 'LineWidth', 1.5);
    title('Aceleración Cartesiana');
    xlabel('Tiempo [s]');
    ylabel('Acel. [m/s^2]');
    legend('A_x', 'A_y', 'A_z', 'Location', 'eastoutside');
    grid on;
end

pause()

%% Cinemática Inversa con Semilla Óptima (Front - Elbow Up)
my_robot;

% q1 apunta al inicio (theta1), q2 inclina hombro adelante, q3 levanta codo
q_semilla_inicial = [theta1, pi/4, -pi/4, 0, pi/2, 0]; 

[Q] = CinematicaInversa(Robot, CPosition, q_semilla_inicial);


%% Configuración del Gráfico de Simulación Realista
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

% 2. SOLUCIÓN AL TRAIL: Dibujamos la trayectoria cartesiana directamente sobre la pieza
% Esto actúa como el cordón de soldadura ya trazado perfectamente en el espacio
plot3(Trayectoria_X, Trayectoria_Y, Trayectoria_Z, 'r-', 'LineWidth', 1);
plot3(Trayectoria_X(1), Trayectoria_Y(1), Trayectoria_Z(1), 'g.', 'MarkerSize', 20); % Inicio en verde

% 3. Graficar la base del robot estática para congelar los ejes y aplicar axis equal
Robot.plot(zeros(1, Robot.n), 'workspace', WS, 'notiles', 'scale', 0.4, 'jointdiam', 0.8);
axis equal; % ¡Vuelve a activarse de forma segura!
view(135, 25);

% 4. Iniciar la animación del brazo robótico recorriendo el cordón
disp('Animando trayectoria en configuración Front-Elbow Up...');
Robot.plot(Q, 'workspace', WS, 'notiles', 'scale', 0.4, 'jointdiam', 0.8, 'fps', 60, 'movie', 'Simulacion_Soldadura');
disp('End of Animation')
