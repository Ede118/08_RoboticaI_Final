function TrayectoriaExterior(guardar_cartesiano, guardar_articular, guardar_jacobiano, guardar_video)
% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

arguments (Input)
    guardar_cartesiano
    guardar_articular
    guardar_jacobiano
    guardar_video
end

%% Path Completo: Posición y Orientación (Soldadura con Weaving e Inclinación)

% Parámetros Especiales
R = 0.8;                    % Radio del cilindro (reducido para no saturar el WorkSpace)
A = 0.02;                   % Amplitud del weaving [m]
A_ang = A / R;              % Amplitud en radianes
k = 20 * 2 * pi;            % Frecuencia de oscilación
alpha_tilt = deg2rad(15);   % Ángulo de avance (15°)

% Posición del Centro del Cilindro respecto a la Base del Robot
Cx = 1.75; % El cilindro está 130 cm frente al robot en el eje X
Cy = 0.0; % Centrado en el eje Y

% Puntos base
z1 = 0.4; z2 = 1.0;
theta1 = pi - pi/6;  % 150 grados
theta2 = pi + pi/6;  % 210 grados
pasos = 1001;

% Ley Temporal
[u, ud, udd] = lspb(0, 1, pasos);

%% Generación de los 4 tramos (Posición y Vector de Avance)

% --- TRAMO 1: Subida ---
theta_T1 = theta1 + A_ang .* sin(k .* u);
Z_T1     = z1 + (z2 - z1) .* u;
X_T1     = Cx + R .* cos(theta_T1);
Y_T1     = Cy + R .* sin(theta_T1);
t_adv_T1 = repmat([0, 0, 1], pasos, 1); 

% --- TRAMO 2: Arco superior ---
theta_T2 = theta1 + (theta2 - theta1) .* u;
Z_T2     = z2 + A .* sin(k .* u);
X_T2     = Cx + R .* cos(theta_T2);
Y_T2     = Cy + R .* sin(theta_T2);
t_adv_T2 = [-sin(theta_T2), cos(theta_T2), zeros(pasos, 1)];

% --- TRAMO 3: Bajada ---
theta_T3 = theta2 + A_ang .* sin(k .* u);
Z_T3     = z2 + (z1 - z2) .* u;
X_T3     = Cx + R .* cos(theta_T3);
Y_T3     = Cy + R .* sin(theta_T3);
t_adv_T3 = repmat([0, 0, -1], pasos, 1);

% --- TRAMO 4: Arco inferior ---
theta_T4 = theta2 + (theta1 - theta2) .* u;
Z_T4     = z1 + A .* sin(k .* u);
X_T4     = Cx + R .* cos(theta_T4);
Y_T4     = Cy + R .* sin(theta_T4);
t_adv_T4 = [sin(theta_T4), -cos(theta_T4), zeros(pasos, 1)];


%% Concatenación de datos espaciales
Trayectoria_X = [X_T1; X_T2; X_T3; X_T4];
Trayectoria_Y = [Y_T1; Y_T2; Y_T3; Y_T4];
Trayectoria_Z = [Z_T1; Z_T2; Z_T3; Z_T4];
Trayectoria_Theta = [theta_T1; theta_T2; theta_T3; theta_T4];
Trayectoria_Adv   = [t_adv_T1; t_adv_T2; t_adv_T3; t_adv_T4];

Total_Pasos = length(Trayectoria_X);

%% ZONA DE EMPALME / BLENDING
% Suavizamos SOLO las transiciones entre tramos, preservando el weaving.

porcentaje_empalme = 0.08;
ancho_empalme = round(pasos * porcentaje_empalme);

% Índices donde un tramo se une con el siguiente
transiciones = [pasos, 2*pasos, 3*pasos];

% Construir máscara de peso: ~1 en la transición, ~0 lejos de ella
peso = zeros(Total_Pasos, 1);
for idx = transiciones
    rango = max(1, idx - ancho_empalme) : min(Total_Pasos, idx + ancho_empalme);
    w = exp(-0.5 * ((rango - idx) / (ancho_empalme / 3)).^2);
    peso(rango) = max(peso(rango), w');
end

% 1. Suavizar Geometría solo en las transiciones (Redondea el vértice sin matar el weaving)
Suave_X = smoothdata(Trayectoria_X, 'gaussian', 2 * ancho_empalme);
Suave_Y = smoothdata(Trayectoria_Y, 'gaussian', 2 * ancho_empalme);
Suave_Z = smoothdata(Trayectoria_Z, 'gaussian', 2 * ancho_empalme);

Trayectoria_X = (1 - peso) .* Trayectoria_X + peso .* Suave_X;
Trayectoria_Y = (1 - peso) .* Trayectoria_Y + peso .* Suave_Y;
Trayectoria_Z = (1 - peso) .* Trayectoria_Z + peso .* Suave_Z;

% 2. Suavizar Orientación solo en las transiciones
Suave_Adv = smoothdata(Trayectoria_Adv, 1, 'gaussian', 2 * ancho_empalme);
Trayectoria_Adv = (1 - peso) .* Trayectoria_Adv + peso .* Suave_Adv;

% Volvemos a normalizar los vectores para que las matemáticas de la rotación no fallen.
Trayectoria_Adv = Trayectoria_Adv ./ vecnorm(Trayectoria_Adv, 2, 2);

%% Cálculo de la Orientación Dinámica [Push Angle]
CPosition = zeros(Total_Pasos, 6); 

for i = 1:Total_Pasos
    th = Trayectoria_Theta(i);
    t_adv = Trayectoria_Adv(i, :);
    
    % a) Vector de Aproximación (Z de la antorcha) = Normal Interior
    a_vec = [-cos(th); -sin(th); 0];
    
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

%% Cinemática Inversa con Semilla Óptima (Front - Elbow Up)
S01_my_robot;

% q1 apunta al inicio (theta1), q2 inclina hombro adelante, q3 levanta codo
q_semilla_inicial = [theta1, pi/4, -pi/4, 0, pi/2, 0]; 

[Q_middle] = CinematicaInversa(Robot, CPosition, q_semilla_inicial);

% --- AGREGADO: Trayectorias de Homing (Aproximación y Retirada) ---
% Se elige q5 = pi/2 para que en el viaje hacia la zona de soldadura (donde q5 ~ pi/2)
% no cruce por q5 = 0 (evitando pasar por la singularidad de muñeca).
q_home = [0, 0, 0, 0, pi/2, 0]; % Posición de Homing (Ready / L-Shape)
pasos_homing = 100; % Cantidad de frames para el viaje desde/hacia homing
tiempo_homing = 10; % Segundos físicos que tarda en acercarse/alejarse

% jtraj calcula una trayectoria suave en el espacio articular
Q_approach = jtraj(q_home, Q_middle(1,:), pasos_homing);
Q_retreat  = jtraj(Q_middle(end,:), q_home, pasos_homing);

% Concatenamos las secuencias sin duplicar el punto de conexión
Q = [Q_approach; Q_middle(2:end, :); Q_retreat(2:end, :)];

fprintf('Matriz Target_Poses (%dx6) generada con éxito.\n', size(Q, 1));

%% Análisis Cinemático en el Espacio Cartesiano - Por Tramo

% 1. Definir el tiempo físico de la trayectoria
tiempo_por_tramo = 60;  % [seg]
t = linspace(0, tiempo_por_tramo, pasos);
dt = t(2) - t(1); 

% 2. Agrupar las coordenadas
Tramos_X = {X_T1, X_T2, X_T3, X_T4};
Tramos_Y = {Y_T1, Y_T2, Y_T3, Y_T4};
Tramos_Z = {Z_T1, Z_T2, Z_T3, Z_T4};
Nombres = {'Tramo 1 (Subida)', 'Tramo 2 (Arco Sup)', 'Tramo 3 (Bajada)', 'Tramo 4 (Arco Inf)'};

% 3. Bucle de cálculo y graficación
carpeta_destino_C = 'Graficos_Cinematica_Cartesiana_OUT';
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

    % Gráfico de Velocidad
    fig2 = figure('Color', 'w', 'Name', ['Velocidad - ' Nombres{i}]);
    plot(t, [Vx Vy Vz], 'LineWidth', 1.5);
    title(['Velocidad Cartesiana - ' Nombres{i}]);
    ylabel('Velocidad [m/s]'); xlabel('Tiempo [s]');
    lgdV = legend('V_x', 'V_y', 'V_z', 'Location', 'eastoutside'); lgdV.ItemHitFcn = @toggleSignal;
    grid on; grid minor;

    % Gráfico de Aceleración
    fig3 = figure('Color', 'w', 'Name', ['Aceleración - ' Nombres{i}]);
    plot(t, [Ax Ay Az], 'LineWidth', 1.5);
    title(['Aceleración Cartesiana - ' Nombres{i}]);
    ylabel('Acel. [m/s^2]'); xlabel('Tiempo [s]');
    lgdA = legend('A_x', 'A_y', 'A_z', 'Location', 'eastoutside'); lgdA.ItemHitFcn = @toggleSignal;
    grid on; grid minor;
    
    if guardar_cartesiano
        nombre_pos = fullfile(carpeta_destino_C, sprintf('Tramo_%d_Posicion.png', i));
        exportgraphics(fig1, nombre_pos, 'Resolution', 300);

        nombre_vel = fullfile(carpeta_destino_C, sprintf('Tramo_%d_Velocidad.png', i));
        exportgraphics(fig2, nombre_vel, 'Resolution', 300);

        nombre_acc = fullfile(carpeta_destino_C, sprintf('Tramo_%d_Aceleracion.png', i));
        exportgraphics(fig3, nombre_acc, 'Resolution', 300);
    end
end

if guardar_cartesiano
    disp('Los 12 gráficos han sido guardados en la carpeta "Graficos_Cinematica".');
end


%% Análisis Cinemático en el Espacio Articular (Motores)

% 1. Definir el tiempo físico
cant_tramos = 4;
tiempo_soldadura = cant_tramos * tiempo_por_tramo; 

% Construimos el vector de tiempo real para coincidir con los 3 bloques de Q
t_app = linspace(0, tiempo_homing, pasos_homing)';
t_mid = linspace(tiempo_homing, tiempo_homing + tiempo_soldadura, size(Q_middle, 1))';
t_ret = linspace(tiempo_homing + tiempo_soldadura, tiempo_homing + tiempo_soldadura + tiempo_homing, pasos_homing)';

t_total = [t_app; t_mid(2:end); t_ret(2:end)];
Total_Pasos_Articulares = length(t_total);

% 2. Inicializar matrices para Velocidad y Aceleración Articular
V_art = zeros(Total_Pasos_Articulares, 6);
A_art = zeros(Total_Pasos_Articulares, 6);

% 3. Cálculo Numérico usando 'gradient' (soporta dt variable para las distintas zonas)
for j = 1:6
    V_art(:, j) = gradient(Q(:, j), t_total);
    A_art(:, j) = gradient(V_art(:, j), t_total);
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

% --- GRÁFICO 2: VELOCIDAD ARTICULAR ---
fig_vq = figure('Color', 'w', 'Name', 'Velocidad Articular');
plot(t_total, rad2deg(V_art), 'LineWidth', 1.5);
title('Evolución de la Velocidad Articular');
ylabel('Velocidad [deg/s]'); xlabel('Tiempo [s]');
lgdQd = legend(nombres_ejes, 'Location', 'eastoutside'); lgdQd.ItemHitFcn = @toggleSignal;
grid on; grid minor;

% --- GRÁFICO 3: ACELERACIÓN ARTICULAR ---
fig_aq = figure('Color', 'w', 'Name', 'Aceleración Articular');
plot(t_total, rad2deg(A_art), 'LineWidth', 1.5);
title('Evolución de la Aceleración Articular');
ylabel('Aceleración [deg/s^2]'); xlabel('Tiempo [s]');
lgdQdd = legend(nombres_ejes, 'Location', 'eastoutside'); lgdQdd.ItemHitFcn = @toggleSignal;
grid on; grid minor;



% --- GRÁFICO 4: DETERMINANTE DEL JACOBIANO ---
disp('Calculando Determinante del Jacobiano...');
det_J = zeros(Total_Pasos_Articulares, 1);
for j = 1:Total_Pasos_Articulares
    J = Robot.jacob0(Q(j,:));
    det_J(j) = det(J);
end

fig_m = figure('Color', 'w', 'Name', 'Determinante del Jacobiano');
plot(t_total, det_J, 'LineWidth', 1.5, 'Color', '#D95319');
title('Determinante del Jacobiano Geométrico (det(J))');
ylabel('det(J)'); xlabel('Tiempo [s]');
grid on; grid minor;

carpeta_destino_Q = 'Graficos_Cinematica_Articular_OUT';
if ~exist(carpeta_destino_Q, 'dir')
    mkdir(carpeta_destino_Q);
end

if guardar_articular
    nombre_q = fullfile(carpeta_destino_Q, 'Articular_1_Posicion.png');
    exportgraphics(fig_q, nombre_q, 'Resolution', 300);

    nombre_vq = fullfile(carpeta_destino_Q, 'Articular_2_Velocidad.png');
    exportgraphics(fig_vq, nombre_vq, 'Resolution', 300);
    
    nombre_aq = fullfile(carpeta_destino_Q, 'Articular_3_Aceleracion.png');
    exportgraphics(fig_aq, nombre_aq, 'Resolution', 300);

    disp('Los 3 gráficos articulares han sido guardados.');
end

if guardar_jacobiano
    nombre_m = fullfile(carpeta_destino_Q, 'Articular_4_Determinante.png');
    exportgraphics(fig_m, nombre_m, 'Resolution', 300);
    disp('Gráfico de Determinante guardado.');
end

disp('Presione [ENTER] para continuar con la simulación.');
pause()


%% Configuración del Gráfico de Simulación Realista
x1lim = -4; x2lim = 4;
y1lim = -4; y2lim = 4;
z1lim = -0.1; z2lim = 2;
WS = [x1lim x2lim y1lim y2lim z1lim z2lim];

figure('Color', 'w', 'Name', 'Simulación de Soldadura Interna', ...
    'WindowStyle', 'normal', 'Units', 'pixels', 'Position', [0 0 1920 1080]); grid on; 
hold on;

% 1. Dibujar el cilindro de la pieza (Desplazado al Centro)
[Xc, Yc, Zc] = cylinder(R, 50);
Xc = Xc + Cx; % Mover la malla en X
Yc = Yc + Cy; % Mover la malla en Y
Zc = Zc * (z2 + 0.2); 
surf(Xc, Yc, Zc, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.4);

% 2. Trayectoria cartesiana directamente sobre la pieza
% Esto actúa como el cordón de soldadura ya trazado perfectamente en el espacio
plot3(Trayectoria_X, Trayectoria_Y, Trayectoria_Z, 'r-', 'LineWidth', 1);
plot3(Trayectoria_X(1), Trayectoria_Y(1), Trayectoria_Z(1), 'g.', 'MarkerSize', 20); % Inicio en verde

% 3. Graficar la base del robot estática para congelar los ejes y aplicar axis equal
axis equal; % ¡Vuelve a activarse de forma segura!

% 4. Iniciar la animación del brazo robótico recorriendo el cordón
disp('Animando trayectoria en configuración Front-Elbow Up...');

if guardar_video
    Robot.plot(...
        Q, ...
        'workspace', WS, ...
        'notiles', ...
        'scale', 0.75, ...
        'jointdiam', 1.5, ...
        'jointlen', 1, ...
        'linkcolor', [.2 .2 .2], ...
        'jointcolor', [1 .4 0], ...
        'fps', 60, ...
        'movie', 'Simulacion_Soldadura_OUT.mp4'...
    );
else
    Robot.plot(...
        Q, ...
        'workspace', WS, ...
        'notiles', ...
        'scale', 0.75, ...
        'jointdiam', 1.5, ...
        'jointlen', 1, ...
        'linkcolor', [.2 .2 .2], ...
        'jointcolor', [1 .4 0], ...
        'fps', 60 ...
    );
end
    
disp('End of Animation')


end