% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/


%% Calculo del Jacobiano Analitico
S01_my_robot;

% 1. Definición de parámetros DH
d = DH(:, 2)*1000;
a = DH(:, 3)*1000;
alpha = DH(:, 4);

% 2. Declaración de variables articulares simbólicas
syms q1 q2 q3 q4 q5 q6
q = [q1, q2, q3, q4, q5, q6];

% 3. Cálculo de la Cinemática Directa de forma simbólica pura
T_acumulada = eye(4);

for i = 1:6
    % Matriz de transformación homogénea estándar de Denavit-Hartenberg
    A_i = [cos(q(i)), -sin(q(i))*cos(alpha(i)),  sin(q(i))*sin(alpha(i)), a(i)*cos(q(i));
        sin(q(i)),  cos(q(i))*cos(alpha(i)), -cos(q(i))*sin(alpha(i)), a(i)*sin(q(i));
        0,          sin(alpha(i)),            cos(alpha(i)),           d(i);
        0,          0,                           0,                          1];

    T_acumulada = T_acumulada * A_i;
end

% Simplificamos la matriz de transformación total homogenea T0_6
T0_6 = simplify(T_acumulada);

% 4. Extracción de las coordenadas operativas de posición (x, y, z)
p_e = T0_6(1:3, 4);
x_pos = p_e(1);
y_pos = p_e(2);
z_pos = p_e(3);

% 5. Extracción de las coordenadas operativas de orientación
% Matriz de rotación simbólica R0_6
R = T0_6(1:3, 1:3);

% Calculo de los ángulos Roll-Pitch-Yaw (Rotaciones extrínsecas en X-Y-Z)
pitch_y = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2)); 
roll_x  = atan2(R(2,1), R(1,1));
yaw_z   = atan2(R(3,2), R(3,3));

% 6. Construcción del vector de coordenadas operativas minimal (6x1 sym)
x_a = [x_pos; y_pos; z_pos; roll_x; pitch_y; yaw_z];


% 7. Cálculo automático del Jacobiano Analítico (Derivadas parciales)
% La función jacobian(f, v) calcula la matriz de derivadas parciales de f respecto a v
fprintf('—————————————— jacobian ——————————————\n\n')
J_analitico = jacobian(x_a, q);

fprintf('—————————————— Simplify Start ——————————————\n\n')

tic;
% 8. Simplificación algebraica final y conversión a formato LaTeX
J = simplify(J_analitico);


% 9. Cálculo del determinante del Jacobiano Analítico

detJ11 = det(J(1:3, 1:3));
detJ12 = det(J(1:3, 4:6));
detJ21 = det(J(4:6, 1:3));
detJ22 = det(J(4:6, 4:6));


% 10. Simplificación algebraica profunda del determinante
% Nota: 'Steps', 50 fuerza a MATLAB a realizar más pasadas de simplificación
detJ11s = simplify(detJ11, 'Steps', 50);
detJ12s = simplify(detJ12, 'Steps', 50);
detJ21s = simplify(detJ21, 'Steps', 50);
detJ22s = simplify(detJ22, 'Steps', 50);

detJ = detJ11s*detJ22s - detJ12s*detJ21s;
detJs = simplify(detJ, 'Steps', 50);

% 11. Conversión del determinante a formato LaTeX para el informe
tiempo_ejecucion = toc;
fprintf('\nTiempo de ejecución: %d\n', tiempo_ejecucion)

fprintf('—————————————— LaTeX Formula ——————————————\n\n')
fprintf('Analitic Jacobian:\n%s\n\n', latex(J))

fprintf('Det J11:\n%s\n\n', latex(detJ11s))
fprintf('Det J22:\n%s\n\n', latex(detJ22s))
fprintf('Det J12:\n%s\n\n', latex(detJ12s))
fprintf('Det J21:\n%s\n\n', latex(detJ21s))

fprintf('Det J:\n%s\n\n', latex(detJs))

% =========================================================================
% Exportación del resultado a un archivo de texto (Modo seguro para toda la noche)
% =========================================================================

disp('Iniciando la conversión a LaTeX y exportación... No cierres MATLAB.');

fid = fopen('det.txt', 'w');
if fid == -1
    error('No se pudo crear o abrir el archivo det.txt. Revisa los permisos de la carpeta.');
end
fprintf(fid, '%s', latex(det_J_simplificado));
fclose(fid);

fid = fopen('Ja.txt', 'w');
if fid == -1
    error('No se pudo crear o abrir el archivo Ja.txt. Revisa los permisos de la carpeta.');
end
fprintf(fid, '%s', latex(J));
fclose(fid);


disp('¡Proceso finalizado con éxito! El resultado se ha guardado en det.txt');