% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

%% Cálculo y Análisis del Jacobiano
clc; clear; close all;

S01_my_robot; Robot.display;

% 1. Definición de parámetros DH 
d = DH(:, 2) * 1000;
a = DH(:, 3) * 1000;
alpha = DH(:, 4);

% 2. Declaración de variables articulares simbólicas
% Agregar 'real' para que MATLAB aplique asserts y simplificaciones
syms q1 q2 q3 q4 q5 q6 real 
q = [q1, q2, q3, q4, q5, q6];

% 3. Inicialización de eslabones y cálculo de matrices acumuladas
T0_i = cell(1, 6);
T_acumulada = eye(4);

for s = 1:6
    % Eslabón simbólico
    L = Revolute('d', d(s), 'a', a(s), 'alpha', alpha(s), 'sym');

    % Matriz de transformación homogénea del eslabón (4x4 sym)
    A_i = L.A(q(s)).double(); 

    % Transformación respecto a la base {0}
    T_acumulada = T_acumulada * A_i;
    T0_i{s} = simplify(T_acumulada);
end

% 4. Extracción del Centro de Muñeca (P_w) desde el Sistema 4
T0_4 = T0_i{4};
p_w = T0_4(1:3, 4);

%% 5. ANÁLISIS DE SINGULARIDADES (Submatriz J11)

% El Jacobiano analítico de la posición P_w coincide exactamente 
% con la submatriz J11 del Jacobiano Geométrico.
J11_analitico = jacobian(p_w, [q1, q2, q3]);

% Determinante
detAJ11 = det(J11_analitico);

% Factorizamos el determinante
vector_factores = factor(detAJ11);

% 'Steps', 50 le dice a MATLAB que intente reducir la ecuación al máximo
detJ11_final = simplify(prod(vector_factores), 'Steps', 50);

%% 6. IMPRESIÓN DE RESULTADOS
disp('——————————————————————————————————————————————————————————————————————')
disp('                   ANÁLISIS DE SINGULARIDADES DE BRAZO                ')
disp('——————————————————————————————————————————————————————————————————————')
fprintf('\nFactores multiplicativos aislados por MATLAB (Ideal para análisis manual):\n\n');

% Imprimir el vector transpuesto (.) para verlo como lista hacia abajo
disp(vector_factores.') 

disp('——————————————————————————————————————————————————————————————————————')
fprintf('Determinante Final Simplificado (Para formato LaTeX):\n\n%s\n\n', latex(detJ11_final))
disp('——————————————————————————————————————————————————————————————————————')