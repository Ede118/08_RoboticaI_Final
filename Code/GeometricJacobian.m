%% Calculo del Jacobiano Desacoplado (Muñeca Esférica)
robot

% 1. Definición de parámetros DH (escalados a mm)
d = DH(:, 2)*1000;
a = DH(:, 3)*1000;
alpha = DH(:, 4);

% 2. Declaración de variables articulares simbólicas
syms q1 q2 q3 q4 q5 q6
q = [q1, q2, q3, q4, q5, q6];

% 3. Inicialización de eslabones y cálculo de matrices acumuladas
T0_i = cell(1, 6);
T_acumulada = eye(4);

for s = 1:6
    L = Revolute('d', d(s), 'a', a(s), 'alpha', alpha(s), 'sym');
    A_i = L.A(q(s)).double(); 
    T_acumulada = T_acumulada * A_i;
    T0_i{s} = simplify(T_acumulada);
end

% 4. Extracción del Centro de la Muñeca Esférica (Origen del sistema 3)
T0_3 = T0_i{3};
p_w = T0_3(1:3, 4); 

% 5. Construcción Geométrica Desacoplada (Evita colgar la PC)
% Inicializamos los vectores para almacenar los ejes Z y posiciones
z = sym(zeros(3, 6));
p = sym(zeros(3, 6));

% El sistema anterior a la primera articulación es la base {0}
z(:, 1) = [0; 0; 1];
p(:, 1) = [0; 0; 0];

% Extraemos los datos para el resto de sistemas acumulados
for i = 2:6
    T_ant = T0_i{i-1};
    z(:, i) = T_ant(1:3, 3); % Ejes Z_0 a Z_5
    p(:, i) = T_ant(1:3, 4); % Orígenes P_0 a P_5
end

% --- Bloque J11: Velocidad lineal del brazo respecto al centro pw ---
J11 = [cross(z(:,1), p_w - p(:,1)), ...
    cross(z(:,2), p_w - p(:,2)), ...
    cross(z(:,3), p_w - p(:,3))];
J11 = simplify(J11);

% --- Bloque J22: Velocidad angular pura de la muñeca ---
J22 = [z(:,4), z(:,5), z(:,6)];
J22 = simplify(J22);

% --- Armado del Jacobiano Geométrico Completo (Estructura Triangular) ---
% El bloque superior derecho es estrictamente CERO gracias a pw
J21 = [z(:,1), z(:,2), z(:,3)]; 
J21 = simplify(J21);

J_geometrico_w = [J11,             sym(zeros(3,3)); 
    J21,             J22];

% 6. Cálculo e Impresión de determinantes independientes
det_Brazo  = simplify(det(J11), 'Steps', 30);
det_Muneca = simplify(det(J22), 'Steps', 30);

fprintf('================ LaTeX Formulas ================\n\n')
fprintf('Jacobiano Geometrico Completo (en pw):\n%s\n\n', latex(J_geometrico_w))
fprintf('Matriz J11 (Brazo):\n%s\n\n', latex(J11))
fprintf('Matriz J22 (Muñeca):\n%s\n\n', latex(J22))
fprintf('Determinante del Brazo (Singularidades del Brazo):\n%s\n\n', latex(det_Brazo))
fprintf('Determinante de la Muñeca (Singularidades de la Muñeca):\n%s\n\n', latex(det_Muneca))