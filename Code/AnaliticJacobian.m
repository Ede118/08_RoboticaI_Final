%% Calculo del Jacobiano Analitico
robot

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
fprintf('================ jacobian ================\n\n')
J_analitico = jacobian(x_a, q);

fprintf('================ Simplify Start ================\n\n')

% 8. Simplificación algebraica final y conversión a formato LaTeX
J_analitico_simplificado = simplify(J_analitico);

% 9. Cálculo del determinante del Jacobiano Analítico
det_J = det(J_analitico_simplificado);

% 10. Simplificación algebraica profunda del determinante
% Nota: 'Steps', 50 fuerza a MATLAB a realizar más pasadas de simplificación
det_J_simplificado = simplify(det_J, 'Steps', 50);

% 11. Conversión del determinante a formato LaTeX para el informe


fprintf('================ LaTeX Formula ================\n\n')
fprintf('Analitic Jacobian:\n%s\n\n', latex(J_analitico_simplificado))
fprintf('Det J:\n%s\n\n', latex(det_J_simplificado))

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
fprintf(fid, '%s', latex(J_analitico_simplificado));
fclose(fid);


disp('¡Proceso finalizado con éxito! El resultado se ha guardado en det.txt');