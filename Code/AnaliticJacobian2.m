%% Cálculo Eficiente del Jacobiano Analítico para el Trabajo Integrador
robot % Se cargan los parámetros DH desde el archivo de configuración robot.m

% 1. Definición de parámetros DH en mm
d = DH(:, 2) * 1000;
a = DH(:, 3) * 1000;
alpha = DH(:, 4);

% 2. Declaración de variables articulares simbólicas en el dominio real
syms q1 q2 q3 q4 q5 q6 real
q = [q1, q2, q3, q4, q5, q6];

% 3. Configuración del modelo cinemático mediante el Robotics Toolbox
% Se instancian los eslabones rotacionales utilizando la convención estándar
L(1) = Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1), 'sym');
L(2) = Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2), 'sym');
L(3) = Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3), 'sym');
L(4) = Revolute('d', d(4), 'a', a(4), 'alpha', alpha(4), 'sym');
L(5) = Revolute('d', d(5), 'a', a(5), 'alpha', alpha(5), 'sym');
L(6) = Revolute('d', d(6), 'a', a(6), 'alpha', alpha(6), 'sym');

% Se construye el objeto de tipo cadena cinemática serie
bot = SerialLink(L, 'name', 'Robot_Mecatronica');

% 4. Obtención de la transformación homogénea total
T0_6 = bot.fkine(q);
R0_6 = T0_6.R; % Extracción de la matriz de rotación simbólica

% 5. Cálculo analítico de los ángulos de orientación Roll-Pitch-Yaw
pitch_y = atan2(-R0_6(3,1), sqrt(R0_6(1,1)^2 + R0_6(2,1)^2)); 
roll_x  = atan2(R0_6(2,1), R0_6(1,1));
yaw_z   = atan2(R0_6(3,2), R0_6(3,3));
angulos_rpy = [roll_x, pitch_y, yaw_z];

% 6. Cálculo del Jacobiano Geométrico con respecto a la base
J_geometrico = bot.jacob0(q);

% 7. Construcción de la matriz de transformación analítica E
% Se obtiene el mapeo de velocidades angulares para la orientación RPY (secuencia ZYX)
T_rpy = rpy2jac(angulos_rpy);

% Se estructura la inversa de la matriz E combinando bloques identidades y la inversa de T_rpy
E_inv = [eye(3), zeros(3); zeros(3), inv(T_rpy)];

% 8. Operación de transformación para la obtención del Jacobiano Analítico
J_analitico = E_inv * J_geometrico;

% 9. Simplificación algebraica única y exportación a archivo de texto
disp('Iniciando la simplificación final. Este proceso demorará pocos segundos.');
J_analitico_simplificado = simplify(J_analitico);

% Se genera el código LaTeX y se almacena en una variable
expresion_latex = latex(J_analitico_simplificado);

% Se realiza la apertura y escritura en el archivo det.txt en el directorio activo
fid = fopen('det.txt', 'w');
if fid ~= -1
    fprintf(fid, '%s', expresion_latex);
    fclose(fid);
    disp('El Jacobiano Analítico ha sido exportado correctamente al archivo det.txt');
else
    disp('Error al intentar escribir en el archivo de texto.');
end