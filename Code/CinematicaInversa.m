function [Q] = CinematicaInversa(R, vector_p, q0)
% CINEMATICAINVERSA Calcula las 8 soluciones articulares (q) para un robot.
% arguments (Input)
%    R          % Estructura o modelo del robot (contiene d, a, alpha, etc.)
%    vector_p   % Vector de pose deseada (Nx6) [x, y, z, roll, pitch, yaw]
% end
% arguments (Output)
%    Q          % Matriz de Nx6x8 con las 8 posibles configuraciones articulares
% end

if size(vector_p, 2) ~= 6
    error('Vector de coordenadas cartesianas debe ser de dimensiones Nx6 [x,y,z,R,P,Y]');
end

if nargin < 3
    q0 = zeros(1, 6);
end

N = size(vector_p, 1);

T_tool = R.tool;
a1 = R.links(1).a; 
a2 = R.links(2).a; 
a3 = R.links(3).a; 

d1 = R.links(1).d; 
d4 = R.links(4).d; 
d6 = R.links(6).d; 

L1 = a2; 
L2 = sqrt(a3^2 + d4^2);
phi = atan2(d4, a3);

% OffSets
offs = zeros(1, 6);
for i = 1:6
    offs(i) = R.links(i).offset;
end


% Pre-alojamiento de memoria (Matriz 3D inicializada en NaN)
QSol = NaN(N, 6, 8);

% Sub-robot para la cinemática de los primeros 3 eslabones
aux_robot = SerialLink(R.links(1:3));

% 4 posturas del brazo:
    % 1: Frente-Arriba 
    % 2: Frente-Abajo 
    % 3: Atrás-Arriba 
    % 4: Atrás-Abajo

map_q1  = [1, 1, 2, 2]; 
map_q23 = [1, 2, 3, 4]; 

%% CICLO PRINCIPAL
for k = 1:N

    % Pose Objetivo (Convención ZYX)
    Target_Pose = SE3(vector_p(k, 1:3)) * SE3.rpy(vector_p(k, 4:6), 'zyx');
    T06_temp = Target_Pose / T_tool; 
    
    % ——————————————————————————————————————— %
    %                   q1                    %
    % ——————————————————————————————————————— %
    
    pos_tcp = T06_temp.t;
    a_tcp   = T06_temp.a;

    p_w = (pos_tcp - a_tcp * d6)';
    px = p_w(1); py = p_w(2); pz = p_w(3);

    if norm(p_w(1:2)) < 1e-6
        warning('Punto %d sobre el eje Z0 (Singularidad de Hombro)', k);
    end

    % Variables temporales, se reinician en cada iteración
    q1_tmp = [atan2(py, px), atan2(-py, -px)];
    q2_tmp = NaN(1, 4);
    q3_tmp = NaN(1, 4);
    
        
    
    % ——————————————————————————————————————— %
    %      Muñeca y Guardado (Las 8 Sol.)     %
    % ——————————————————————————————————————— %
    
    for rama = 1:4
        idx_1  = map_q1(rama);
        idx_23 = map_q23(rama);

        % Solo calculamos la muñeca si el brazo alcanzó la posición
        if ~isnan(q2_tmp(idx_23)) 
            
            
            q_brazo = [q1_tmp(idx_1), q2_tmp(idx_23), q3_tmp(idx_23)];
            q_brazo_real = q_brazo - offs(1:3); 
            
            % Cinemática directa parcial 
            T03_temp = aux_robot.fkine(q_brazo_real);
            R03 = T03_temp.R; 
            R06 = T06_temp.R;
            
            r_mat = R03' * R06;

            % --- Solución 1: Wrist Up (No-Flip) ---
            q5_up = atan2(norm([r_mat(1, 3), r_mat(2, 3)]), r_mat(3, 3));
            q4_up = atan2(r_mat(2, 3), r_mat(1, 3));
            q6_up = atan2(r_mat(3, 2), -r_mat(3, 1));
            
            % Guardar vector completo de 6 juntas en la capa impar (1, 3, 5, 7)
            col_WristUp = 2*rama - 1; 
            QSol(k, :, col_WristUp) = [q_brazo, q4_up, q5_up, q6_up];

            % --- Solución 2: Wrist Down (Flip) ---
            q5_down = atan2(-norm([r_mat(1, 3), r_mat(2, 3)]), r_mat(3, 3));
            q4_down = atan2(-r_mat(2, 3), -r_mat(1, 3)); 
            q6_down = atan2(-r_mat(3, 2), r_mat(3, 1)); 

            % Guardar vector completo de 6 juntas en la capa par (2, 4, 6, 8)
            col_WristDown = 2*rama;
            QSol(k, :, col_WristDown) = [q_brazo, q4_down, q5_down, q6_down];
        end
    end
end

%%
% —————————————————————————————————————————————————————— %
% 1. APLICACIÓN DE OFFSETS Y WRAP (Vectorizado)
% —————————————————————————————————————————————————————— %

offs_3d = reshape(offs, 1, 6, 1);
QSol = wrapToPi(QSol - offs_3d);

%%
% —————————————————————————————————————————————————————— %
% 2. FILTRADO POR LÍMITES ARTICULARES (Vectorizado)
% —————————————————————————————————————————————————————— %

for joint = 1:6
    q_min = R.links(joint).qlim(1);
    q_max = R.links(joint).qlim(2);
    
    % Posturas que violan los limites
    mascara_invalida = QSol(:, joint, :) < q_min | QSol(:, joint, :) > q_max;
    
    % Si una articulación falla, invalidamos la fila entera (las 6 juntas)
    mascara_completa = repmat(mascara_invalida, 1, 6, 1);
    QSol(mascara_completa) = NaN;
end

%%
% —————————————————————————————————————————————————————— %
% 3. SELECCIÓN DE LA SOLUCIÓN ÓPTIMA
% —————————————————————————————————————————————————————— %

Q = zeros(N, 6); 

% -- Evaluación del Punto Inicial (k = 1) --
dist_min = inf;
mejor_sol_inicial = 0;

for sol = 1:8
    q_candidato = QSol(1, :, sol);
    
    if ~any(isnan(q_candidato)) 
        % Distancia en el espacio articular respecto a la postura inicial
        distancia = norm(wrapToPi(q_candidato - q0));
        
        if distancia < dist_min
            dist_min = distancia;
            mejor_sol_inicial = sol;
        end
    end
end

if mejor_sol_inicial == 0
    error('El punto inicial es inalcanzable. Revisa el espacio de trabajo o los límites articulares.');
end

Q(1, :) = QSol(1, :, mejor_sol_inicial);

% -- Rastreo Continuo para el resto de la trayectoria (k = 2:N) --
for k = 2:N
    q_previo = Q(k-1, :);
    dist_min = inf;
    mejor_sol = 0; 

    for sol = 1:8
        q_candidato = QSol(k, :, sol);
        
        if any(isnan(q_candidato))
            continue;
        end
        
        % Minimizamos el salto articular para asegurar continuidad
        dist = norm(wrapToPi(q_candidato - q_previo)); 
        
        if dist < dist_min
            dist_min = dist;
            mejor_sol = sol;
        end
    end
    
    if mejor_sol == 0
        warning('El punto %d es inalcanzable. El robot se detendrá en la postura anterior.', k);
        Q(k, :) = q_previo; 
    else
        Q(k, :) = QSol(k, :, mejor_sol);
    end
end

end