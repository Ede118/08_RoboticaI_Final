% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/


%% SCRIPT DE VALIDACIÓN: CINEMÁTICA INVERSA
clear; clc; close all;

% 1. Cargar el robot (Asegúrate de que 'S01_my_robot' define la variable 'Robot')
S01_my_robot; 

fprintf('Iniciando Test de Cinemática Inversa...\n\n');

%% =========================================================================
% TEST 1: PUNTOS ALEATORIOS DENTRO DEL ESPACIO DE TRABAJO
% =========================================================================
fprintf('--- TEST 1: 100 Puntos Aleatorios ---\n');
N_test = 100;
q_test_rand = zeros(N_test, 6);

% Generar ángulos aleatorios respetando los límites articulares
for j = 1:6
    q_min = Robot.links(j).qlim(1);
    q_max = Robot.links(j).qlim(2);
    q_test_rand(:, j) = q_min + (q_max - q_min) * rand(N_test, 1);
end

% Construir el vector_p de entrada
vector_p_rand = zeros(N_test, 6);
Target_poses = Robot.fkine(q_test_rand);

for k = 1:N_test
    vector_p_rand(k, 1:3) = Target_poses(k).t';
    % Extraer Roll, Pitch, Yaw en convención ZYX
    vector_p_rand(k, 4:6) = Target_poses(k).tr2rpy('zyx'); 
end

% Cronometrar
tic;
Q_calc_rand = CinematicaInversa(Robot, vector_p_rand);
tiempo_ejecucion = toc;

% Evaluar Error Cartesiano
error_pos_max = 0;
for k = 1:N_test
    T_calc = Robot.fkine(Q_calc_rand(k, :));
    e_pos = norm(Target_poses(k).t - T_calc.t) * 1000; % En milímetros
    if e_pos > error_pos_max
        error_pos_max = e_pos;
    end
end

fprintf('Tiempo de cálculo para %d puntos: %.4f segundos.\n', N_test, tiempo_ejecucion);
fprintf('Error máximo de posición: %g mm\n', error_pos_max);
if error_pos_max < 1e-6
    fprintf('Resultado: EXCELENTE (Error atribuible a precisión de máquina).\n\n');
else
    fprintf('Resultado: FALLÓ (Revisar ecuaciones).\n\n');
end


%% =========================================================================
% TEST 2: EL LÍMITE DEL ESPACIO DE TRABAJO (SINGULARIDAD DE BORDE)
% =========================================================================
fprintf('--- TEST 2: Brazo completamente estirado ---\n');
% Valores articulares que estiren el brazo al maximo
q_estirado = [0, 0, 0, 0, 0, 0]; 

T_estirado = Robot.fkine(q_estirado);
v_estirado = [T_estirado.t', T_estirado.tr2rpy('zyx')];

Q_calc_estirado = CinematicaInversa(Robot, v_estirado);
T_calc_estirado = Robot.fkine(Q_calc_estirado(1, :));

e_pos_estirado = norm(T_estirado.t - T_calc_estirado.t) * 1000;
fprintf('Error de posición al límite: %g mm\n\n', e_pos_estirado);


%% =========================================================================
% TEST 3: FUERA DEL ESPACIO DE TRABAJO (ROBUSTEZ)
% =========================================================================
fprintf('--- TEST 3: Coordenada Inalcanzable ---\n');
% Pedimos un punto inalcanzable (distancia ~17 [m])
vector_p_imposible = [10, 10, 10, 0, 0, 0]; 

try
    Q_imposible = CinematicaInversa(Robot, vector_p_imposible);
    disp('La función se ejecutó. Verificando manejo de error silencioso...');
catch ME
    fprintf('La función detectó correctamente el error y se detuvo.\n');
    fprintf('Mensaje de tu función: "%s"\n', ME.message);
    fprintf('Resultado: EXCELENTE (Manejo de excepciones robusto).\n');
end

