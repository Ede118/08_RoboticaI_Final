%% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

%clear; clc; close all;
S01_my_robot;

%% Mostrar sistemas de referencia {S_i}
q = zeros(1, Robot.n);
L = 0.35;

figure('Color','w'); grid on; axis equal;
view(135,25);

% Opciones:
% - 'nowrist', ...
% -
% -

Robot.plot(q, ...
    'workspace', WS, ...
    'notiles', ...
    'scale', 0.75, ...
    'jointdiam', 1.5, ...
    'jointlen', 1, ...
    'linkcolor', [.2 .2 .2], ...
    'jointcolor', [1 .4 0]);
Robot.teach();
hold on;

colores = [0.000, 0.278, 0.671; 0.843, 0.000, 0.251; 0.314, 0.784, 0.471; 1.000, 0.749, 0.000; 0.502, 0.000, 0.502];

% Opcion de Graficar el robot con los sistemas de coordenada
if false
    % ----- Grafico de {S_i} -----
    T = Robot.base;
    for m = 1:Robot.n
        if (show_axis(m) == 1) && m == 1
            trplot(Robot.base, 'frame','0', 'length', L);
        end
        if (show_axis(m) == 1) && m > 1
            Ai = Robot.A(m-1, q);
            T  = T * Ai;
            if m <= length(show_axis) && show_axis(m) == 1
                trplot(T, 'frame', ...
                    num2str(m-1), ...
                    'length', L, ...
                    'width', 0.5,...
                    'thick', 10,...
                    'axis', [-2 2 -2 2 -0.2 2], ...
                    'rgb', ...
                    'arrow');
            end
        end
    end
    
    % (Optional) Mostrar el tool como {T}
    if true
        trplot(T * Robot.tool, 'frame','T', 'length', L);
        title('Frames seleccionados');
    end
end


%% Análisis del Espacio de Trabajo (Plano XY y XZ)

disp('Calculando envolvente principal R-Z...');

% Límites articulares de la base, hombro y codo
q1_lim = Robot.qlim(1, :);
q2_lim = Robot.qlim(2, :);
q3_lim = Robot.qlim(3, :);

% Malla (Plano R-Z base)
resolucion = 150;
[Q2, Q3] = meshgrid(linspace(q2_lim(1), q2_lim(2), resolucion), ...
    linspace(q3_lim(1), q3_lim(2), resolucion));
q2_vec = Q2(:); q3_vec = Q3(:);
N = length(q2_vec);

% Parámetros DH
d = Robot.d;
a = Robot.a;
al = Robot.alpha;

R = zeros(N, 1); Z = zeros(N, 1);
for i = 1:N
    T06 = eye(4);
    Q = [0, q2_vec(i), q3_vec(i), 0, 0, 0];
    for j = 1:6
        th = Q(j);
        A = [cos(th), -sin(th)*cos(al(j)),  sin(th)*sin(al(j)), a(j)*cos(th);
            sin(th),  cos(th)*cos(al(j)), -cos(th)*sin(al(j)), a(j)*sin(th);
            0,        sin(al(j)),         cos(al(j)),         d(j);
            0,        0,                  0,                  1];
        T06 = T06 * A;
    end
    R(i) = T06(1,4); Z(i) = T06(3,4);
end

% Frontera 2D (polígono)
k = boundary(R, Z, 0.75);
R_b = R(k); 
Z_b = Z(k);

disp('Proyectando hacia los planos canónicos...');

% --- Plano XY (Sector Anular) ---
R_max = max(R_b);
R_min = min(R_b(R_b > 0)); % Radio mínimo de alcance
if isempty(R_min), R_min = 0; end

% Barrido de q1 
theta_sweep = linspace(q1_lim(1), q1_lim(2), 100); 
X_out = R_max * cos(theta_sweep);
Y_out = R_max * sin(theta_sweep);

% fliplr invierte el vector interior para cerrar el polígono correctamente
X_in = R_min * cos(fliplr(theta_sweep)); 
Y_in = R_min * sin(fliplr(theta_sweep));

X_XY = [X_out, X_in]; % Contorno cerrado en X
Y_XY = [Y_out, Y_in]; % Contorno cerrado en Y

% --- Plano XZ (Corte Frontal) ---

% Lóbulo derecho (q1 = 0 grados -> X = R)
X_XZ_der = R_b;
Z_XZ_der = Z_b;

% Lóbulo izquierdo (q1 = -170 grados -> X = R * cos(-170))
% Multiplicarlo por cos(q1) refleja la silueta respetando su límite físico.

X_XZ_izq = R_b * cos(q1_lim(1)); 
Z_XZ_izq = Z_b;

% --- GRAFICO ---
figure('Color', 'w', 'Name', 'Envolventes XY', 'Position', [150 150 1000 500]);

% Límites Físicos
% Definimos una zona de exclusión cilíndrica (ej. el robot no puede tocar su base)
% Radio mínimo de seguridad (la base del robot tiene cierto radio)
R_limit = 0.25; 
Z_limit = -0.05;
validos = (R > R_limit) & (Z > Z_limit);

R_f = R(validos);
Z_f = Z(validos);

k_RZ = boundary(R_f, Z_f, 0.75);

% Gráfico 1: Vista de Planta (XY)
% subplot(1,2,1); 
hold on; grid on; grid minor;
fill(X_XY, Y_XY, [0.85 0.9 0.98], 'FaceAlpha', 0.8, 'EdgeColor', [0 0.3 0.6], 'LineWidth', 2);
plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2); % Centro de la base
title('Área de Trabajo - Vista de Planta (Plano XY)');
xlabel('Eje X [m]'); ylabel('Eje Y [m]');
axis equal;

% Gráfico 2: Vista Frontal (XZ)
figure('Color', 'w', 'Name', 'Envolventes XZ', 'Position', [150 150 1000 500]);
% subplot(1,2,2); 
hold on; grid on; grid minor;
% Pintamos la silueta derecha e izquierda
fill(X_XZ_der, Z_XZ_der, [0.85 0.9 0.98], 'FaceAlpha', 0.8, 'EdgeColor', [0 0.3 0.6], 'LineWidth', 2);

% Detalles de la base
plot([0 0], [0 0.450], 'k-', 'LineWidth', 3); % Pedestal (d1)
plot(0, 0, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); 
title('Área de Trabajo - Vista Frontal (Plano XZ)');
xlabel('Eje X [m]'); ylabel('Eje Z [m]');
axis equal;

disp('¡Proyecciones completadas!');