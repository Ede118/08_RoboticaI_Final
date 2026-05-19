%% Cálculo del Jacobiano Analítico Optimizado
robot

% 1. Definición de parámetros DH
d = DH(:, 2) * 1000;
a = DH(:, 3) * 1000;
alpha = DH(:, 4);

% 2. Declaración de variables articulares simbólicas
syms q1 q2 q3 q4 q5 q6 real
q = [q1, q2, q3, q4, q5, q6];

% 3. Construcción del robot utilizando el Robotics Toolbox (Peter Corke)
L(1) = Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1), 'sym');
L(2) = Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2), 'sym');
L(3) = Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3), 'sym');
L(4) = Revolute('d', d(4), 'a', a(4), 'alpha', alpha(4), 'sym');
L(5) = Revolute('d', d(5), 'a', a(5), 'alpha', alpha(5), 'sym');
L(6) = Revolute('d', d(6), 'a', a(6), 'alpha', alpha(6), 'sym');

bot = SerialLink(L, 'name', 'Robot_6DOF');

% 4. Cálculo del Jacobiano Geométrico con respecto a la base (Frame 0)
% La función jacob0 está optimizada internamente y evita las singularidades por atan2
J_geometrico = bot.jacob0(q);

% 5. Simplificación algebraica final y conversión a formato LaTeX
% Al no arrastrar simplificaciones iterativas, este proceso concluye rápidamente
J_analitico_simplificado = simplify(J_geometrico);

% Mostrar en ventana de comandos la cadena de texto para el informe
latex(J_analitico_simplificado)