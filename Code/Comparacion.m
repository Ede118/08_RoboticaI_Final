% SCRIPT DE DEPURACIÓN DE CINEMÁTICA INVERSA

% 1. Posición articular conocida
q_prueba = [0.5, pi/4, -pi/4, 0.1, pi/3, 0.2]; 

% 2. Usa la cinemática directa de tu objeto R para obtener la matriz objetivo
T_target = R.fkine(q_prueba); 

if isobject(T_target), T_target = T_target.T; end

[Rot06, p06] = tr2rt(T_target);
angles = tr2rpy(T_target, 'xyz')';
x = [p06; angles];

q_soluciones = CinematicaInversa(R, x); 

% 4. Imprime en consola para comparar
disp('--- Q ORIGINAL DE PRUEBA ---');
disp(q_prueba);

disp('--- Q CALCULADOS (Revisa si alguna fila se parece a la original) ---');
disp(q_soluciones');