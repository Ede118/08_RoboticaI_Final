%[text] # Pruebas de Cinemática Directa
S01_my_robot; 

N_test = 100;
q_test = zeros(N_test, 6);
for j = 1:6
    q_min = Robot.links(j).qlim(1);
    q_max = Robot.links(j).qlim(2);
    q_test(:, j) = q_min + (q_max - q_min) * rand(N_test, 1);
end

XSol_manual = CinematicaDirecta(q_test, Robot);

T_toolbox = Robot.fkine(q_test);
XSol_toolbox = zeros(N_test, 6);

for k = 1:N_test
    XSol_toolbox(k, 1:3) = T_toolbox(k).t';
    XSol_toolbox(k, 4:6) = T_toolbox(k).tr2rpy('zyx');
end

% Comparar resultados respecto al Toolbox
error_pos = max(abs(XSol_manual(:, 1:3) - XSol_toolbox(:, 1:3)));
error_ori = max(abs(wrapToPi(XSol_manual(:, 4:6) - XSol_toolbox(:, 4:6))));

fprintf('Error máximo en posición (x, y, z): [%g, %g, %g] m\n', error_pos(1), error_pos(2), error_pos(3));
fprintf('Error máximo en orientación (rpy): [%g, %g, %g] rad\n', error_ori(1), error_ori(2), error_ori(3));

if norm(error_pos) < 1e-12 && norm(error_ori) < 1e-12
    fprintf('\n[ACEPTABLE]. \n');
else
    fprintf('\n[FAIL]: El resultado propio difiere significativamente del resultado del Toolbox.\n');
end


mostrar_poses = true;

q_pose1 = deg2rad([35.9, -29.4, 13.3, 75.7, -59.1, 120]);
x_pose_xyz1 = [-0.799, 0.997, 1.007, deg2rad(8.025), deg2rad(-73.688), deg2rad(160.844)];

q_pose2 = deg2rad([-99.7, -22.1, -57.2, 81.7, 63.3, -15]);
x_pose_xyz2 = [0.888, -0.219, 0.346, deg2rad(94.209), deg2rad(7.268), deg2rad(112.195)];

q_pose3 = deg2rad([00.0, -97.4, 41.2, -1.51, 49.2, 0]);
x_pose_xyz3 = [-0.001, 1.367, -0.160, deg2rad(-90.984), deg2rad(-1.141), deg2rad(-86.981)];

q_pose4 = deg2rad([-95.7, 0, 0, 0, 0, 0]);
x_pose_xyz4 = [0.965, -0.097, 1.395, deg2rad(150.044), deg2rad(78.497), deg2rad(29.456)];

x_pose_xyz = [x_pose_xyz1; x_pose_xyz2; x_pose_xyz3; x_pose_xyz4];

x_pose = zeros(4, 6);
for k = 1:4
    Tk = SE3(x_pose_xyz(k, 1:3)) * SE3.rpy(x_pose_xyz(k, 4:6), 'xyz');
    x_pose(k, :) = [Tk.t', Tk.tr2rpy('zyx')];
end

if mostrar_poses

figure('Name', 'Posicion de Ejemplo 1','Color','w'); grid on; grid minor; axis equal; view(135,25);
Robot.plot(q_pose1, ...
    'workspace', WS, ...
    'notiles', ...
    'scale', 0.75, ...
    'jointdiam', 1.5, ...
    'jointlen', 1, ...
    'linkcolor', [.2 .2 .2], ...
    'jointcolor', [1 .4 0]);
legend(sprintf('TCP: [%.3f, %.3f, %.3f] m\nRPY (xyz): [%.1f, %.1f, %.1f]°', ...
    x_pose_xyz(1, 1), x_pose_xyz(1, 2), x_pose_xyz(1, 3), ...
    rad2deg(x_pose_xyz(1, 4)), rad2deg(x_pose_xyz(1, 5)), rad2deg(x_pose_xyz(1, 6))), ...
    'Location', 'northwest');
set(findobj(gcf, 'Tag', Robot.name), 'Tag', ''); % Evita que el toolbox reutilice esta figura

figure('Name', 'Posicion de Ejemplo 2','Color','w'); grid on; grid minor; axis equal; view(135,25);
Robot.plot(q_pose2, ...
    'workspace', WS, ...
    'notiles', ...
    'scale', 0.75, ...
    'jointdiam', 1.5, ...
    'jointlen', 1, ...
    'linkcolor', [.2 .2 .2], ...
    'jointcolor', [1 .4 0]);
legend(sprintf('TCP: [%.3f, %.3f, %.3f] m\nRPY (xyz): [%.1f, %.1f, %.1f]°', ...
    x_pose_xyz(2, 1), x_pose_xyz(2, 2), x_pose_xyz(2, 3), ...
    rad2deg(x_pose_xyz(2, 4)), rad2deg(x_pose_xyz(2, 5)), rad2deg(x_pose_xyz(2, 6))), ...
    'Location', 'northwest');
set(findobj(gcf, 'Tag', Robot.name), 'Tag', ''); % Evita que el toolbox reutilice esta figura


WS(5) = -1;

figure('Name', 'Posicion de Ejemplo 3','Color','w'); grid on; grid minor; axis equal; view(135,25);
Robot.plot(q_pose3, ...
    'workspace', WS, ...
    'notiles', ...
    'scale', 0.75, ...
    'jointdiam', 1.5, ...
    'jointlen', 1, ...
    'linkcolor', [.2 .2 .2], ...
    'jointcolor', [1 .4 0]);
legend(sprintf('TCP: [%.3f, %.3f, %.3f] m\nRPY (xyz): [%.1f, %.1f, %.1f]°', ...
    x_pose_xyz(3, 1), x_pose_xyz(3, 2), x_pose_xyz(3, 3), ...
    rad2deg(x_pose_xyz(3, 4)), rad2deg(x_pose_xyz(3, 5)), rad2deg(x_pose_xyz(3, 6))), ...
    'Location', 'northwest');
set(findobj(gcf, 'Tag', Robot.name), 'Tag', ''); % Evita que el toolbox reutilice esta figura

WS(5) = -0.1;

figure('Name', 'Posicion de Ejemplo 4','Color','w'); grid on; grid minor; axis equal; view(135,25);
Robot.plot(q_pose4, ...
    'workspace', WS, ...
    'notiles', ...
    'scale', 0.75, ...
    'jointdiam', 1.5, ...
    'jointlen', 1, ...
    'linkcolor', [.2 .2 .2], ...
    'jointcolor', [1 .4 0]);
legend(sprintf('TCP: [%.3f, %.3f, %.3f] m\nRPY (xyz): [%.1f, %.1f, %.1f]°', ...
    x_pose_xyz(4, 1), x_pose_xyz(4, 2), x_pose_xyz(4, 3), ...
    rad2deg(x_pose_xyz(4, 4)), rad2deg(x_pose_xyz(4, 5)), rad2deg(x_pose_xyz(4, 6))), ...
    'Location', 'northwest');
set(findobj(gcf, 'Tag', Robot.name), 'Tag', ''); % Evita que el toolbox reutilice esta figura

end

% Error en las poses, respecto al Toolbox
my_x_pose = zeros(4, 6);
my_x_pose(1, :) = CinematicaDirecta(q_pose1, Robot);
my_x_pose(2, :) = CinematicaDirecta(q_pose2, Robot);
my_x_pose(3, :) = CinematicaDirecta(q_pose3, Robot);
my_x_pose(4, :) = CinematicaDirecta(q_pose4, Robot);

for k = 1:4
    % Convertimos la pose calculada (en zyx) a matriz SE3 para cambiar de convención
    T_calc = SE3(my_x_pose(k, 1:3)) * SE3.rpy(my_x_pose(k, 4:6), 'zyx');
    my_rpy_xyz = T_calc.tr2rpy('xyz');
    
    error_pos = abs(my_x_pose(k, 1:3) - x_pose_xyz(k, 1:3));
    error_ori = abs(wrapToPi(my_rpy_xyz - x_pose_xyz(k, 4:6)));

    fprintf('\n--- CONFIGURACIÓN %d ---\n', k);
    fprintf('Error en posición (x, y, z): [%g, %g, %g] m\n', error_pos(1), error_pos(2), error_pos(3));
    fprintf('Error en orientación (rpy xyz): [%g, %g, %g] rad\n', error_ori(1), error_ori(2), error_ori(3));
end

%%
%[text] # Pruebas de Cinemática Directa
%[text] ## Seeds Initialization
clear; close all; clc;
S01_my_robot; Robot.display;

N = 1001;
angles = linspace(-pi, pi, N);
xlimit = [-pi, pi];
lbls = {'-\pi', '-3\pi/4', '-\pi/2', '-\pi/4', '0', '\pi/4', '\pi/2', '3\pi/4', '\pi'};

QSolution = zeros(N, 6);

% Values [-π, π]
for k = 1:6
    QSolution(:, k) = -pi + (2*pi/(N-1)) * (0:N-1)';
end

% Random values
% for k = 1:6
%     QSolution(:, k) = -pi + 2*pi * rand(N, 1);
% end

hat = Robot.offset';
Robot.offset = zeros(6, 1);

Target = Robot.fkine(QSolution);

T_tool = Robot.tool;

a1 = Robot.links(1).a; 
a2 = Robot.links(2).a; 
a3 = Robot.links(3).a; 

d1 = Robot.links(1).d; 
d4 = Robot.links(4).d; 
d6 = Robot.links(6).d; 

L1 = a2; 
L2 = sqrt(a3^2 + d4^2);



T06   = repmat(SE3(), N, 1);
T03   = repmat(SE3(), N, 1);
p_w   = zeros(N, 3);

q1    = zeros(N, 2);
q2    = zeros(N, 4);
q3    = zeros(N, 4);

q4    = zeros(N, 2);
q5    = zeros(N, 2);
q6    = zeros(N, 2);

err_q1    = zeros(N, 2);
err_q2    = zeros(N, 4);
err_q3    = zeros(N, 4);
err_q4    = zeros(N, 2);
err_q5    = zeros(N, 2);
err_q6    = zeros(N, 2);

phi = atan2(a3, d4); 
%[text] ## Solution $q\_1$
for k = 1:N
    T06(k) = Target(k) / T_tool; 
    
    % Kinematic Decoupling:
    % p_w = p - d6 * z
    pos_tcp = T06(k).t;
    z_tcp   = T06(k).a;
    
    p_w(k, :) = (pos_tcp - z_tcp * d6)';
    
    if norm(p_w(k, 1:2)) < 1e-6
        disp('Advertencia: Punto sobre el eje Z0 (Singularidad)')
    end
    
    q1(k, 1) = atan2(p_w(k, 2), p_w(k, 1));
    q1(k, 2) = atan2(-p_w(k, 2), -p_w(k, 1));
end


err_q1(:, 1) = rad2deg(((QSolution(:, 1) - q1(:, 1))));
err_q1(:, 2) = rad2deg(((QSolution(:, 1) - q1(:, 2))));

figure('Name', "Error q1"); 
plot(angles, [err_q1(:, 1) err_q1(:, 2)], 'LineWidth', 1);

title('Error Absoluto q_1'); xlabel('Ángulo [rad]'); ylabel('Error Relativo [deg]');

lgd = legend('e_{1}', 'e_{2}', 'Location', 'northeastoutside');
lgd.ItemHitFcn = @toggleSignal;

xlim(xlimit);
set(gca, 'XTick', -pi : pi/4 : pi);
set(gca, 'XTickLabel', lbls, 'TickLabelInterpreter', 'tex');

grid on;grid minor;
%[text] ## Solution $q\_2, q\_3$
for k = 1:N
    r     = zeros(2, 1);
    D     = zeros(2, 1);
    alpha = zeros(2, 1);

    px = p_w(k, 1);
    py = p_w(k, 2);
    pz = p_w(k, 3);

    % Front Configuration
    r(1) =  sqrt(px^2 + py^2) - a1; 
    % Back Configuration
    r(2) = -sqrt(px^2 + py^2) - a1;
    
    s = pz - d1; 


    for j = 1:2
        D(j)     = sqrt(r(j)^2 + s^2);
        alpha(j) = atan2(s, r(j));
        
        % j=1 -> idx1=1, idx2=2
        % j=2 -> idx1=3, idx2=4
        
        idx1 = 2*j - 1;
        idx2 = 2*j;

        if D(j) <= (L1 + L2) && D(j) >= abs(L1 - L2)
            beta  = Tcoseno_a(L1, L2, D(j));
            gamma = Tcoseno_a(D(j), L1, L2);

            % Elbow Up
            q2(k, idx1) = alpha(j) + gamma;
            q3(k, idx1) = beta - pi/2 - phi;

            % Elbow Down
            q2(k, idx2) = alpha(j) - gamma;
            q3(k, idx2) = -beta + pi/2 - phi;
        else
            q2(k, idx1:idx2) = NaN;
            q3(k, idx1:idx2) = NaN;
        end
    end
end

err_q2(:, 1) = rad2deg((QSolution(:, 2) - q2(:, 1)));
err_q2(:, 2) = rad2deg((QSolution(:, 2) - q2(:, 2)));
err_q2(:, 3) = rad2deg((QSolution(:, 2) - q2(:, 3)));
err_q2(:, 4) = rad2deg((QSolution(:, 2) - q2(:, 4)));

figure('Name', "Error q2"); 

plot(angles, [err_q2(:, 1) err_q2(:, 2) err_q2(:, 3) err_q2(:, 4)], 'LineWidth', 1);

title('Error Absoluto q_2'); xlabel('Ángulo [rad]'); ylabel('Error Relativo [deg]');

lgd2 = legend('e_{1}', 'e_{2}', 'e_{3}', 'e_{4}', 'Location', 'northeastoutside');
lgd2.ItemHitFcn = @toggleSignal;

xlim(xlimit);
set(gca, 'XTick', -pi : pi/4 : pi);
set(gca, 'XTickLabel', lbls, 'TickLabelInterpreter', 'tex');


grid on;grid minor;

err_q3(:, 1) = rad2deg((QSolution(:, 3) - q3(:, 1)));
err_q3(:, 2) = rad2deg((QSolution(:, 3) - q3(:, 2)));
err_q3(:, 3) = rad2deg((QSolution(:, 3) - q3(:, 3)));
err_q3(:, 4) = rad2deg((QSolution(:, 3) - q3(:, 4)));

figure('Name', "Error q3"); 

plot(angles, [err_q3(:, 1) err_q3(:, 2) err_q3(:, 3) err_q3(:, 4)], 'LineWidth', 1);

title('Error Absoluto q_3'); xlabel('Ángulo [rad]'); ylabel('Error Relativo [deg]');
lgd3 = legend('e_{1}', 'e_{2}', 'e_{3}', 'e_{4}', 'Location', 'northeastoutside');
lgd3.ItemHitFcn = @toggleSignal;

xlim(xlimit);
set(gca, 'XTick', -pi : pi/4 : pi);
set(gca, 'XTickLabel', lbls, 'TickLabelInterpreter', 'tex');

grid on;grid minor;
%[text] ## Solution $q\_4, q\_5, q\_6$
% Robot with only the first 3 Links
robot_sub = SerialLink(Robot.links(1:3));

for k = 1:N
    % Testing articular vector, only used to test the logic
    % for the I.K. code
    q = [QSolution(k, 1), QSolution(k, 2), QSolution(k, 3)];

    % F.K. Robot with only the first 3 Links
    T03(k) = robot_sub.fkine(q);

    R03 = T03(k).R; R06 = T06(k).R;
    
    % Roation Matrix R36
    r = R03 \ R06; 

    % Wrist Up
    q5(k, 1) = atan2(norm([r(1, 3), r(2, 3)]), r(3, 3));
    q4(k, 1) = atan2(r(2, 3), r(1, 3));
    q6(k, 1) = atan2(r(3, 2), -r(3, 1));

    % Wrist Down
    q5(k, 2) = atan2(-norm([r(1, 3), r(2, 3)]), r(3, 3));
    q4(k, 2) = atan2(-r(2, 3), -r(1, 3)); 
    q6(k, 2) = atan2(-r(3, 2), r(3, 1)); 
end

err_q4(:, 1) = rad2deg((QSolution(:, 4) - q4(:, 1)));
err_q4(:, 2) = rad2deg((QSolution(:, 4) - q4(:, 2)));

figure('Name', "Error q4"); 

plot(angles, [err_q4(:, 1) err_q4(:, 2)], 'LineWidth', 1);

title('Error Absoluto q_4'); xlabel('Ángulo [rad]'); ylabel('Error Relativo [deg]');
lgd4 = legend('e_{1}', 'e_{2}', 'Location', 'northeastoutside');
lgd4.ItemHitFcn = @toggleSignal;

xlim(xlimit);
set(gca, 'XTick', -pi : pi/4 : pi);
set(gca, 'XTickLabel', lbls, 'TickLabelInterpreter', 'tex');

grid on;grid minor;

err_q5(:, 1) = rad2deg((QSolution(:, 5) - q5(:, 1)));
err_q5(:, 2) = rad2deg((QSolution(:, 5) - q5(:, 2)));

figure('Name', "Error q5"); 

plot(angles, [err_q5(:, 1) err_q5(:, 2)], 'LineWidth', 1);

title('Error Absoluto q_5'); xlabel('Ángulo [rad]'); ylabel('Error Relativo [deg]');
lgd5 = legend('e_{1}', 'e_{2}', 'Location', 'northeastoutside');
lgd5.ItemHitFcn = @toggleSignal;

xlim(xlimit);
set(gca, 'XTick', -pi : pi/4 : pi);
set(gca, 'XTickLabel', lbls, 'TickLabelInterpreter', 'tex');

grid on;grid minor;

err_q6(:, 1) = rad2deg((QSolution(:, 6) - q6(:, 1)));
err_q6(:, 2) = rad2deg((QSolution(:, 6) - q6(:, 2)));

figure('Name', "Error q6"); 

plot(angles, [err_q6(:, 1) err_q6(:, 2)], 'LineWidth', 1);

title('Error Absoluto q_6'); xlabel('Ángulo [rad]'); ylabel('Error Relativo [deg]');
lgd6 = legend('e_{1}', 'e_{2}', 'Location', 'northeastoutside');
lgd6.ItemHitFcn = @toggleSignal;

xlim(xlimit);
set(gca, 'XTick', -pi : pi/4 : pi);
set(gca, 'XTickLabel', lbls, 'TickLabelInterpreter', 'tex');

grid on;grid minor;
%%
%[text] # Validacion de Cinematica Inversa
clear; clc; close all;
S01_my_robot; 
%[text] ## TEST 1: PUNTOS ALEATORIOS DENTRO DEL ESPACIO DE TRABAJO
%[text] 
fprintf('--- TEST 1: 100 Puntos Aleatorios ---\n'); %[output:67e1a21b]
N_test = 100;
q_test_rand = zeros(N_test, 6);

% Generar ángulos aleatorios respetando los límites articulares
for j = 1:6
    q_min = Robot.links(j).qlim(1);
    q_max = Robot.links(j).qlim(2);
    q_test_rand(:, j) = q_min + (q_max - q_min) * rand(N_test, 1);
end

% Construir el vector_p de entrada con metodo del Toolbox
% vector_p_rand = zeros(N_test, 6);
% Target_poses = Robot.fkine(q_test_rand);
% 

% for k = 1:N_test
%     vector_p_rand(k, 1:3) = Target_poses(k).t';
%     % Extraemos Roll, Pitch, Yaw en convención ZYX
%     vector_p_rand(k, 4:6) = Target_poses(k).tr2rpy('zyx'); 
% end

% Construir el vector_p de entrada con la Cinematica Directa desarrollada

vector_p_rand   = CinematicaDirecta(q_test_rand, Robot);
pos             = vector_p_rand(:, 1:3);
angles          = vector_p_rand(:, 4:6);
q0              = zeros(1, 6);
% Probar la función
tic;
Q_calc_rand = CinematicaInversa(Robot, vector_p_rand, q0, true);
tiempo_ejecucion = toc;

% Evaluar Error Cartesiano
error_pos_max = 0;
error_ori_max = 0;
for k = 1:N_test
    T_calc = Robot.fkine(Q_calc_rand(k, :));
    e_pos = norm(pos(k, :) - T_calc.t) * 1000; % En milímetros
    if e_pos > error_pos_max
        error_pos_max = e_pos;
    end
    
    e_ori = norm(angles(k, :) - T_calc.tr2rpy('zyx')); % En milímetros
    if e_ori > error_ori_max
        error_ori_max = e_ori;
    end
    
end

fprintf('Tiempo de cálculo para %d puntos: %.4f segundos.\n', N_test, tiempo_ejecucion); %[output:0dc19a7f]
fprintf('Error máximo de posición: %g [mm]\n', error_pos_max); %[output:30473866]
fprintf('Error máximo de orientación: %g [rad]\n', error_pos_max); %[output:1784aae2]
if error_pos_max < 1e-6 %[output:group:68b88762]
    fprintf('Posicion: [SUCCESS].\n\n');
else
    fprintf('Posicion: [FAIL].\n\n'); %[output:248b8192]
end %[output:group:68b88762]
if error_ori_max < 1e-6 %[output:group:5ffd64a5]
    fprintf('Orientacion: [SUCCESS].\n\n'); %[output:7dc12b12]
else
    fprintf('Orientacion: [FAIL].\n\n');
end %[output:group:5ffd64a5]
%[text] ## TEST 2: EL LÍMITE DEL ESPACIO DE TRABAJO (SINGULARIDAD DE BORDE)
% Buscamos la posicion donde deje el brazo estirado
x_estirado = Robot.a(1) + Robot.a(2) + Robot.d(4) + Robot.d(6);
y_estirado = 0;
z_estirado = Robot.d(1) + Robot.a(3);

p_estirado = [x_estirado, y_estirado, z_estirado, 0, 0, 0]; q0 = zeros(1, 6);
T_estirado = SE3(x_estirado, y_estirado, z_estirado);

q_toolbox   = Robot.ikine6s(T_estirado); %[output:8b4e68a2]
q_propio    = CinematicaInversa(Robot, p_estirado, q0, true);

T_toolboX   = Robot.fkine(q_toolbox);

estirado    = CinematicaDirecta(q_propio, Robot);
angulos     = SE3.rpy(estirado(4), estirado(5), estirado(6), 'zyx');

e_pos_estirado = norm(T_toolbox.t - estirado(1:3)) * 1000;
fprintf('Error de posición al límite: %g [mm]\n\n', e_pos_estirado);

e_pos_estirado = norm(T_toolbox.t - estirado(1:3)) * 1000;
fprintf('Error de orientacion al límite: %g [10e3 rad]\n\n', e_pos_estirado);

%[text] ## TEST 3: FUERA DEL ESPACIO DE TRABAJO (ROBUSTEZ)
% Pedimos un punto a 10 metros de distancia (imposible para este robot)
vector_p_imposible = [10, 10, 10, 0, 0, 0]; 

try
    Q_imposible = CinematicaInversa(Robot, vector_p_imposible);
    % Si la función no se detiene, comprobamos si devolvió NaNs o Ceros.
    disp('La función se ejecutó. Verificando manejo de error silencioso...');
    % Nota: Según tu código, debería saltar el error del 'all(Q(1,:)==0)'
catch ME
    fprintf('La función detectó correctamente el error y se detuvo.\n');
    fprintf('Mensaje de tu función: "%s"\n', ME.message);
    fprintf('Resultado: EXCELENTE (Manejo de excepciones robusto).\n');
end

%%
%[text] ## Jacobian Unit Test
clear; close all; S01_my_robot; Robot.display;

%%
%[text] ## Save Variables
% writematrix(err_q4, 'error_q4.csv');
% writematrix(err_q3, 'error_q3.csv');
% writematrix(error12, 'error12.csv');
% writematrix(error21, 'error21.csv');
% writematrix(error22, 'error22.csv');
% writematrix(QSolution, 'QSolution.csv');

%[text] ## 
%[text] 
%[text] 

%[appendix]{"version":"1.0"}
%---
%[metadata:styles]
%   data: {"heading1":{"color":"#268cdd"},"referenceBackgroundColor":"#ffffff"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":60.2}
%---
%[output:67e1a21b]
%   data: {"dataType":"text","outputData":{"text":"--- TEST 1: 100 Puntos Aleatorios ---\n","truncated":false}}
%---
%[output:0dc19a7f]
%   data: {"dataType":"text","outputData":{"text":"Tiempo de cálculo para 100 puntos: 0.5638 segundos.\n","truncated":false}}
%---
%[output:30473866]
%   data: {"dataType":"text","outputData":{"text":"Error máximo de posición: 2887.11 [mm]\n","truncated":false}}
%---
%[output:1784aae2]
%   data: {"dataType":"text","outputData":{"text":"Error máximo de orientación: 2887.11 [rad]\n","truncated":false}}
%---
%[output:248b8192]
%   data: {"dataType":"text","outputData":{"text":"Posicion: [FAIL].\n\n","truncated":false}}
%---
%[output:7dc12b12]
%   data: {"dataType":"text","outputData":{"text":"Orientacion: [SUCCESS].\n\n","truncated":false}}
%---
%[output:8b4e68a2]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('SerialLink\/ikine6s', '\/home\/barrios14101\/GitHub\/08_RoboticaI_Final\/Toolbox\/rvctools\/robot\/@SerialLink\/ikine6s.m', 137)\" style=\"font-weight:bold\">SerialLink\/ikine6s<\/a> (<a href=\"matlab: opentoline('\/home\/barrios14101\/GitHub\/08_RoboticaI_Final\/Toolbox\/rvctools\/robot\/@SerialLink\/ikine6s.m',137,0)\">line 137<\/a>)\nThis kinematic structure not supported"}}
%---
