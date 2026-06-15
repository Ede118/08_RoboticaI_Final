%% Mapeo de la Submatriz J11 con Intersección Z=0
clc; close all;

my_robot; 

f = @(q2, q3) 16000 * ( 140*cosd(q3) - 39*sind(q3) ) .* ( 39*cosd(q2+q3) + 140*sind(q2+q3) + 128*cosd(q2) + 15 );

% C = [3.36e7; 8.736e7; 2.4336e7; 9.36e6; 2.8672e8;7.9872e7; 1.7472e8; 2.89264e8;];
% 
% f = @(q2, q3) C(1)*cosd(q3) ...
%     - C(2)*cosd(q2) ...
%     + C(3)*sind(q2) ...
%     - C(4)*sind(q3) ...
%     + C(5)*cosd(q2).*cosd(q3) ...
%     - C(6)*cosd(q2).*sind(q3) ...
%     + C(7)*cosd(q2).*cosd(q3).^2 ...
%     + C(8)*(sind(q2).*cosd(q3).^2 + cosd(q2).*cosd(q3).*sind(q3)) ...
%     - C(7)*cosd(q3).*sind(q2).*sind(q3);
%
% Sacado de la expresion:
% \begin{align}
% \det (\mathbf{J}_{11}) & = 33.600.000\,\cos\left(q_{3}\right) \\
% & -87.360.000\,\cos\left(q_{2}\right) \\
% & +24.336.000\,\sin\left(q_{2}\right) \\
% & -9.360.000\,\sin\left(q_{3}\right) \\
% & +286.720.000\,\cos\left(q_{2}\right)\,\cos\left(q_{3}\right) \\
% & -79.872.000\,\cos\left(q_{2}\right)\,\sin\left(q_{3}\right) \\
% & +174.720.000\,\cos\left(q_{2}\right)\,{\cos\left(q_{3}\right)}^2 \\
% & +289.264.000\,{\cos\left(q_{3}\right)}^2\,\sin\left(q_{2}\right) \\
% & +289.264.000\,\cos\left(q_{2}\right)\,\cos\left(q_{3}\right)\,\sin\left(q_{3}\right) \\
% & -174.720.000\,\cos\left(q_{3}\right)\,\sin\left(q_{2}\right)\,\sin\left(q_{3}\right)
% \end{align}


figure('Name', 'Determinante J11')

% Subplot 1: Superficie 3D automática
subplot(1, 2, 1)
fsurf(f, [-360, 360, -360, 360]) 
title('Superficie 3D con fsurf')
xlabel('q2'); ylabel('q3'); zlabel('det')
grid on

% Subplot 2: Mapa de contorno automático
subplot(1, 2, 2)
fcontour(f, [-360, 360, -360, 360], 'Fill', 'on') 
hold on  

fc = fcontour(f, [-360, 360, -360, 360], 'LevelList', 0, 'LineColor', 'r', 'LineWidth', 2.5);

title('Líneas de Singularidad Física (det = 0)')
xlabel('q2'); ylabel('q3')
colorbar
hold off

drawnow; 

% Matriz de contorno
C = fc.ContourMatrix;

idx = 1;
valores_singulares = [];

while idx < size(C, 2)
    num_puntos = C(2, idx);
    q2_segmento = C(1, idx+1 : idx+num_puntos);
    q3_segmento = C(2, idx+1 : idx+num_puntos);
    
    
    valores_singulares = [valores_singulares; q2_segmento', q3_segmento'];
    
    % Siguiente segmento
    idx = idx + num_puntos + 1;
end

disp('Algunos pares de [q2, q3] donde el determinante es cero:');
disp(valores_singulares(1:10, :)); % Muestra los primeros 10

indices_prueba = round(linspace(1, size(valores_singulares, 1), 10));
puntos_prueba = valores_singulares(indices_prueba, :);

% Fijamos el resto de las articulaciones. 
% ¡CRÍTICO! q5 = pi/2 para que sin(q5) ~= 0 y evitar la singularidad de muñeca.
q1 = 0; 
q4 = 0; 
q5 = pi/2;
q6 = 0;

disp('det J11 con fcontour (q2, q3):');
fprintf('   q2 (deg)  |   q3 (deg)  |  Determinante (det J)\n');
fprintf('———————————————————————————————————————————————————————\n');

for i = 1:size(puntos_prueba, 1)
    % 1. Extraemos el valor del gráfico (matemático) en radianes
    q2_math = deg2rad(puntos_prueba(i, 1)); 
    q3_math = deg2rad(puntos_prueba(i, 2));

    % 2. Le RESTAMOS el offset real del robot para compensar lo que el Toolbox suma
    q2_test = q2_math - Robot.links(2).offset;
    q3_test = q3_math - Robot.links(3).offset;

    % Armamos la postura completa
    q_test = [q1, q2_test, q3_test, q4, q5, q6];

    % Calculamos el Jacobiano respecto a la base
    J = Robot.jacob0(q_test);
    det_J = det(J);

    % Mostramos el resultado
    fprintf('%12.4f | %12.4f | %e\n', puntos_prueba(i,1), puntos_prueba(i,2), det_J);
end


% =========================================================================
% REFINAMIENTO NUMÉRICO CON FSOLVE
% =========================================================================
pause(10);
fprintf('———————————————————————————————————————————————————————\n');
fprintf('———————————————————————————————————————————————————————\n');

disp('Refinando puntos de contorno con fsolve para encontrar el cero exacto...');

% 1. Adaptamos la función para que reciba un vector x = [q2, q3]
f_num = @(x) 16000 * ( 140*cosd(x(2)) - 39*sind(x(2)) ) .* ...
    ( 39*cosd(x(1)+x(2)) + 140*sind(x(1)+x(2)) + 128*cosd(x(1)) + 15 );

% 2. Configuramos fsolve para máxima precisión y que no imprima texto basura
options = optimoptions('fsolve', 'Display', 'off', ...
    'FunctionTolerance', 1e-14, ...
    'StepTolerance', 1e-14);

puntos_exactos = zeros(size(puntos_prueba));

for i = 1:size(puntos_prueba, 1)
    % Usamos el punto del gráfico como "adivinanza" inicial
    q_guess = [puntos_prueba(i, 1), puntos_prueba(i, 2)];

    % fsolve busca la raíz exacta más cercana
    q_exact = fsolve(f_num, q_guess, options);

    % Guardamos el punto refinado
    puntos_exactos(i, :) = q_exact;
end

disp('Refinamiento completado.');
disp(' ');

% =========================================================================
% VERIFICACIÓN FINAL CON EL JACOBIANO DEL TOOLBOX
% =========================================================================
% Fijamos el resto de las articulaciones (q5 = pi/2 para evitar muñeca)
q1 = 0; 
q4 = 0; 
q5 = pi/2;
q6 = 0;

disp('det J11 (q2, q3) con PUNTOS EXACTOS:');
fprintf('   q2 (deg)  |   q3 (deg)  |  Determinante (det J)\n');
fprintf('--------------------------------------------------\n');

for i = 1:size(puntos_exactos, 1)
    % 1. Extraemos el valor EXACTO en radianes
    q2_math = deg2rad(puntos_exactos(i, 1)); 
    q3_math = deg2rad(puntos_exactos(i, 2));

    % 2. Le RESTAMOS el offset real del robot
    q2_test = q2_math - Robot.links(2).offset;
    q3_test = q3_math - Robot.links(3).offset;

    % Armamos la postura completa
    q_test = [q1, q2_test, q3_test, q4, q5, q6];

    % Calculamos el Jacobiano respecto a la base
    J = Robot.jacob0(q_test);
    det_J = det(J);

    % Mostramos el resultado
    fprintf('%12.4f | %12.4f | %e\n', puntos_exactos(i,1), puntos_exactos(i,2), det_J);
end