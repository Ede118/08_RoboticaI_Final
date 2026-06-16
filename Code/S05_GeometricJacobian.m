% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

%% Calculo del Jacobiano
clc;

S01_my_robot; Robot.display;

% 1. Definición de parámetros DH
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
    % Eslabón simbólico
    L = Revolute('d', d(s), 'a', a(s), 'alpha', alpha(s), 'sym');
    
    % Matriz de transformación homogénea del eslabón (4x4 sym)
    A_i = L.A(q(s)).double(); 
    
    % Transformación respecto a la base {0}
    T_acumulada = T_acumulada * A_i;
    T0_i{s} = simplify(T_acumulada);
end

% 4. Extracción del punto del efector final
T0_6 = T0_i{6};
p_e = T0_6(1:3, 4); 

T0_4 = T0_i{4};
p_w = T0_4(1:3, 4);

% 5. Construcción geométrica de la matriz Jacobiana (6x6 sym)
J = sym(zeros(6, 6));

for i = 1:6
    if i == 1
        % Para la primera articulación, el sistema anterior es la base {0}
        z_ant = [0; 0; 1];
        p_ant = [0; 0; 0];
    else
        % Para las siguientes, extraemos los datos de T0_{i-1}
        T_ant = T0_i{i-1};
        z_ant = T_ant(1:3, 3); % Eje Z del sistema anterior
        p_ant = T_ant(1:3, 4); % Origen del sistema anterior
    end
    
    % Aplicación de las ecuaciones del Jacobiano Geométrico
    J(1:3, i) = cross(z_ant, p_w - p_ant); % Submatriz lineal (velocidad lineal)
    J(4:6, i) = z_ant;                     % Submatriz angular (velocidad angular)
end

% 6. Simplificación algebraica y conversión a formato LaTeX
J_simplified = simplify(J);


J11 = J(1:3, 1:3);
J12 = J(1:3, 4:6);
J21 = J(4:6, 1:3);
J22 = J(4:6, 4:6);

detJ11 = det(J11);
detJ12 = det(J12);
detJ21 = det(J21);
detJ22 = det(J22);

detJ11_Simplified = simplify(detJ11, 'Steps', 50);
detJ11_Simplified = simplify(detJ11_Simplified, 'Steps', 50);
detJ12_Simplified = simplify(detJ12);
detJ21_Simplified = simplify(detJ21);
detJ22_Simplified = simplify(detJ22);

fprintf('—————————————— LaTeX Formula ——————————————\n\n')
fprintf('Geometric Jacobian:\n%s\n\n', latex(J_simplified))
fprintf('J11:\n%s\n\n', latex(J11))
fprintf('Det J11:\n%s\n\n', latex(detJ11_Simplified))
fprintf('J22:\n%s\n\n', latex(J22))
fprintf('Det J22:\n%s\n\n', latex(detJ22_Simplified))
fprintf('Det J12:\n%s\n\n', latex(detJ12_Simplified))
fprintf('Det J21:\n%s\n\n', latex(detJ21_Simplified))



