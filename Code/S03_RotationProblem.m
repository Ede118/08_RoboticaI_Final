% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/


%% Calculo de 3R6
S01_my_robot

d = DH(4:6, 2)*1000;
a = DH(4:6, 3)*1000;
alpha = DH(4:6, 4);

% Declaración de variables articulares simbólicas
syms q4 q5 q6 

q = [q4, q5, q6];

% Inicialización del vector de eslabones cinemáticos
L = Link.empty(3,0);

% Creación de los eslabones con parámetros simbólicos
for s = 1:3
    L(s) = Revolute('d', d(s), 'a', a(s), 'alpha', alpha(s), 'sym');
end

% Evaluación analítica de las matrices de transformación homogénea (clase SE3)
T3_4 = L(1).A(q(1));
T4_5 = L(2).A(q(2));
T5_6 = L(3).A(q(3));

% Composición de la muñeca esférica completa
T3_6 = T3_4 * T4_5 * T5_6;

% Extracción de las matrices simbólicas y conversión a formato LaTeX
matrix_sym = T3_6.double();
matrix_simplified = simplify(matrix_sym);

latex(matrix_simplified)


