function [Q] = CinematicaInversa(R, vector_p, q0, random_pos)
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

if nargin < 4
   random_pos = false; 
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

% offsets
offs = zeros(1, 6);
for i = 1:6
    offs(i) = R.links(i).offset;
end


% armo matriz de salidas llena de NaNs
QSol = NaN(N, 6, 8);

% separo los primeros 3 eslabones para la cinematica de posicion
aux_robot = SerialLink(R.links(1:3));

% 4 posturas del brazo:
% 1: frente-arriba 
% 2: frente-abajo 
% 3: atras-arriba 
% 4: atras-abajo

map_q1  = [1, 1, 2, 2]; 
map_q23 = [1, 2, 3, 4]; 

%% iteracion punto a punto
for k = 1:N

    % paso la pose objetivo a matriz de transformacion homogenea
    Target_Pose = SE3(vector_p(k, 1:3)) * SE3.rpy(vector_p(k, 4:6), 'zyx');
    T06_temp = Target_Pose / T_tool; 
    
    % ——————————————————————————————————————— %
    %                   q1                    %
    % ——————————————————————————————————————— %
    
    pos_tcp = T06_temp.t;
    a_tcp   = T06_temp.a;

    % vector desde la muñeca al TCP
    p_w = (pos_tcp - a_tcp * d6)';
    px = p_w(1); py = p_w(2); pz = p_w(3);

    if norm(p_w(1:2)) < 1e-6
        warning('Punto %d sobre el eje Z0 (Singularidad de Hombro)', k);
    end

    % reseteo las variables temporales
    q1_tmp = [atan2(py, px), atan2(-py, -px)];
    q2_tmp = NaN(1, 4);
    q3_tmp = NaN(1, 4);
    
    % ——————————————————————————————————————— %
    %                q2 y q3                  %
    % ——————————————————————————————————————— %

    % r(1): frontal
    % r(2): de espalda

    r = [ sqrt(px^2 + py^2) - a1;
         -sqrt(px^2 + py^2) - a1 ];
    s = pz - d1;

    for j = 1:2
        D     = sqrt(r(j)^2 + s^2);
        alpha = atan2(s, r(j));

        idx1 = 2*j - 1; % codo arriba
        idx2 = 2*j;     % codo abajo

        if D <= (L1 + L2 + 1e-6) && D >= (abs(L1 - L2) - 1e-6)
            % arreglo el lado D por si hay error numerico
            D_seguro = max(min(D, L1+L2), abs(L1-L2));

            % aplico el teorema del coseno
            beta  = Tcoseno_a(L1, L2, D_seguro);
            gamma = Tcoseno_a(D_seguro, L1, L2);

            % elbow up
            q2_tmp(idx1) = alpha + gamma;
            q3_tmp(idx1) = (beta - pi) + phi;

            % elbow down
            q2_tmp(idx2) = alpha - gamma;
            q3_tmp(idx2) = -(beta - pi) + phi;
        end
    end
    
    % ——————————————————————————————————————— %
    %      Muñeca y Guardado (Las 8 Sol.)     %
    % ——————————————————————————————————————— %
    
    for rama = 1:4
        idx_1  = map_q1(rama);
        idx_23 = map_q23(rama);

        % saco orientacion solo si el brazo pudo llegar al punto
        if ~isnan(q2_tmp(idx_23)) 
            
            
            q_brazo = [q1_tmp(idx_1), q2_tmp(idx_23), q3_tmp(idx_23)];
            q_brazo_real = q_brazo - offs(1:3); 
            
            % saco la rotacion de los primeros 3 ejes
            T03_temp = aux_robot.fkine(q_brazo_real);
            R03 = T03_temp.R; 
            R06 = T06_temp.R;
            
            % lo que le falta rotar a la muñeca
            r_mat = R03' * R06;

            % solucion 1: wrist up (sin flip)
            q5_up = atan2(norm([r_mat(1, 3), r_mat(2, 3)]), r_mat(3, 3));
            q4_up = atan2(r_mat(2, 3), r_mat(1, 3));
            q6_up = atan2(r_mat(3, 2), -r_mat(3, 1));
            
            % guardo la solucion en la capa impar
            col_WristUp = 2*rama - 1; 
            QSol(k, :, col_WristUp) = [q_brazo, q4_up, q5_up, q6_up];

            % solucion 2: wrist down (con flip)
            q5_down = atan2(-norm([r_mat(1, 3), r_mat(2, 3)]), r_mat(3, 3));
            q4_down = atan2(-r_mat(2, 3), -r_mat(1, 3)); 
            q6_down = atan2(-r_mat(3, 2), r_mat(3, 1)); 

            % guardo la solucion en la capa par
            col_WristDown = 2*rama;
            QSol(k, :, col_WristDown) = [q_brazo, q4_down, q5_down, q6_down];
        end
    end
end

%% offsets y mapeo al rango [-pi, pi]

offs_3d = reshape(offs, 1, 6, 1);
QSol = wrapToPi(QSol - offs_3d);

%% filtro soluciones que chocan con los limites del robot

for joint = 1:6
    q_min = R.links(joint).qlim(1);
    q_max = R.links(joint).qlim(2);
    
    % me fijo que configuraciones se pasan de los limites
    mascara_invalida = QSol(:, joint, :) < q_min | QSol(:, joint, :) > q_max;
    
    % si falla una junta, descarto toda esa configuracion entera (las 6)
    mascara_completa = repmat(mascara_invalida, 1, 6, 1);
    QSol(mascara_completa) = NaN;
end

%% elijo la solucion optima para que el robot se mueva lo menos posible

Q = zeros(N, 6); 

% arranco evaluando la pose inicial 
dist_min = inf;
mejor_sol_inicial = 0;

for sol = 1:8
    q_candidato = QSol(1, :, sol);
    
    if ~any(isnan(q_candidato)) 
        % distancia articular respecto a la semilla (q0)
        distancia = norm(wrapToPi(q_candidato - q0));
        
        if distancia < dist_min
            dist_min = distancia;
            mejor_sol_inicial = sol;
        end
    end
end

if mejor_sol_inicial == 0
    if random_pos
        Q(1, :) = NaN(1, 6);
    else
        error('El punto inicial es inalcanzable. Revisa el espacio de trabajo o los límites articulares.');
    end
else
    Q(1, :) = QSol(1, :, mejor_sol_inicial);
end

% recorro el resto de los puntos (para trayectoria continua o puntos sueltos)
for k = 2:N
    if random_pos
        q_previo = q0;
    else
        q_previo = Q(k-1, :);
    end
    dist_min = inf;
    mejor_sol = 0; 

    for sol = 1:8
        q_candidato = QSol(k, :, sol);
        
        if any(isnan(q_candidato))
            continue;
        end
        
        % busco la que este mas cerquita del punto anterior (asi no da un salto loco)
        dist = norm(wrapToPi(q_candidato - q_previo)); 
        
        if dist < dist_min
            dist_min = dist;
            mejor_sol = sol;
        end
    end
    
    if mejor_sol == 0
        if random_pos
            Q(k, :) = NaN(1, 6);
        else
            warning('El punto %d es inalcanzable. El robot se detendrá en la postura anterior.', k);
            Q(k, :) = q_previo;
        end
    else
        Q(k, :) = QSol(k, :, mejor_sol);
    end
end

end