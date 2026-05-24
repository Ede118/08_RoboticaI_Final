function [q] = CinematicaInversa(R, vector_p)
    % CINEMATICAINVERSA Calcula las 8 soluciones articulares (q) para un robot.
    arguments (Input)
        R           % Estructura o modelo del robot (contiene d, a, alpha, etc.)
        vector_p    % Vector de pose deseada (6xK) [x, y, z, roll, pitch, yaw]
    end
    arguments (Output)
        q           % Matriz de 6x8 con las 8 posibles configuraciones articulares
    end
    
    if size(vector_p, 1) ~= 6
        disp('Error: size(vector_p) no tiene 6 filas.')        
        return
    end
    
    T_target = x2T(vector_p); 
    
    if isa(R.tool, 'SE3')
        T_tool = R.tool.T;
    else
        T_tool = R.tool;
    end
    
    T06 = T_target / T_tool; 
    
    p_w = T06(1:3, 4) - T06(1:3, 3) * R.d(6); % (3,1)
    
    q = zeros(6,8);
    
    %% 1. Soluciones para q1 (Hombro)
    if (p_w(1) == 0) && (p_w(2) == 0)
        disp('Advertencia: Punto sobre el eje Z0 (Singularidad)')
    end
    
    q1_front = atan2(p_w(2), p_w(1));
    q1_back  = wrapToPi(q1_front + pi);
    
    q(1, 1:4) = q1_front;
    q(1, 5:8) = q1_back;
    
    %% Despeje de variables intermedias para q2 y q3
    A1 = A_DH(q(1,1), R.d(1), R.a(1), R.alpha(1));
    Rot1 = A1(1:3, 1:3);
    v1 = A1(1:3, 4);
    
    T01 = rt2tr(Rot1, v1);
    
    % Distancia desde el origen de la articulación 2 hasta el centro de la muñeca
    dif = p_w - T01(1:3, 4);
    
    
    % Se proyecta el vector en el plano de elevación (Articulaciones 2 y 3)
    distancia_radial = norm(dif(1:2)); % Cuánto se aleja horizontalmente
    distancia_vertical = dif(3);       % Cuánto sube o baja verticalmente
    
    % vec_r ahora es un vector [2x1] en el plano local del brazo
    vec_r = [distancia_radial; distancia_vertical];
    
    % Ahora 'r' sí representa la hipotenusa 3D real desde el hombro a la muñeca
    r = norm(vec_r); 
    
    H2 = R.a(3)^2 + R.d(4)^2;
    H = sqrt(H2);
    
    
    beta    = Tcoseno_a(R.a(2), r, H);
    gamma   = Tcoseno_a(R.a(2), H, r);

    %% Para 1ra y 2da solución de q1 (Codo arriba / Codo abajo)
    q(2, 1:2) = atan2(vec_r(2), vec_r(1)) + beta;
    q(3, 1) = pi - gamma;
    q(3, 2) = -pi + gamma;
    q(2, 3:4) = atan2(vec_r(2), vec_r(1)) - beta;
    q(3, 3) = pi - gamma;
    q(3, 4) = -pi + gamma;
    
    q(2, 5:6) = atan2(vec_r(2), vec_r(1)) + beta;
    q(3, 5) = pi - gamma;
    q(3, 6) = -pi + gamma;
    q(2, 7:8) = atan2(vec_r(2), vec_r(1)) - beta;
    q(3, 7) = pi - gamma;
    q(3, 8) = -pi + gamma;
    
    %% Despeje de la Muñeca Esférica (q4, q5, q6)
    for i = 1:8
        A01 = A_DH(q(1,i), R.d(1), R.a(1), R.alpha(1));
        A12 = A_DH(q(2,i), R.d(2), R.a(2), R.alpha(2));
        A23 = A_DH(q(3,i), R.d(3), R.a(3), R.alpha(3));
        
        T03 = A01 * A12 * A23;
        R03T = T03(1:3, 1:3)'; % Transpuesta de la rotación (inversa)
        
        Rot36 = R03T * T06(1:3, 1:3);
        
        q(4,i) = atan2(Rot36(2,3), Rot36(1,3));
        
        if mod(i, 2) ~= 0
            q(5,i) = atan2(sqrt(Rot36(1,3)^2 + Rot36(2,3)^2), Rot36(3,3));
        else
            q(5,i) = atan2(-sqrt(Rot36(1,3)^2 + Rot36(2,3)^2), Rot36(3,3));
        end
        
        q(6,i) = atan2(-Rot36(3,2), Rot36(3,1));
    end
    
    % Envolver ángulos entre -pi y pi
    q = wrapToPi(q);
    
    if isfield(R, 'offset')
        robot_offset = R.offset;
    else
        robot_offset = zeros(1, 6); % Valor por defecto si no existe offset
    end
    
    robot_offset = R.offset'


    for s = 1:6
        q(s, :) = q(s, :) - robot_offset(s) * ones(1, 8);
    end
    
    q = wrapToPi(q);
end

%% Subfunciones Auxiliares

function [T] = A_DH(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

function [T] = x2T(x)
    % Convierte un vector de pose (6x1) en una matriz de transformación (4x4)
    if size(x,1) ~= 6
        disp('Advertencia: x debe ser de 6 filas')
    end
    
    pos = x(1:3, :);
    Rot = rpy2r(x(4:6, :)', 'xyz'); 
    
    T = rt2tr(Rot, pos);
end

function angulo_rad = Tcoseno_a(lado_a, lado_b, lado_opuesto)
    % TEOREMA_COSENO_ANGULO Calcula el ángulo opuesto al 'lado_opuesto'.
    % Entradas:
    %   lado_a, lado_b : Longitudes de los lados adyacentes al ángulo a buscar.
    %   lado_opuesto   : Longitud del lado opuesto al ángulo a buscar.
    % Salidas:
    %   angulo_rad     : Ángulo resultante en radianes.
    
    numerador = lado_a^2 + lado_b^2 - lado_opuesto^2;
    denominador = 2 * lado_a * lado_b;
    
    coseno_angulo = numerador / denominador;
    
    if coseno_angulo > 1
        coseno_angulo = 1;
    elseif coseno_angulo < -1
        coseno_angulo = -1;
    end
    
    angulo_rad = acos(coseno_angulo);
end

function lado_resultante = Tcoseno_l(lado_a, lado_b, angulo_rad)
    % TEOREMA_COSENO_LADO Calcula un lado desconociendo dado el ángulo opuesto.
    % Entradas:
    %   lado_a, lado_b : Longitudes de los lados conocidos.
    %   angulo_rad     : Ángulo entre lado_a y lado_b (en radianes).
    % Salidas:
    %   lado_resultante: Longitud del lado opuesto al ángulo dado.
    
    % Fórmula: c^2 = a^2 + b^2 - 2ab*cos(gamma)
    lado_cuadrado = lado_a^2 + lado_b^2 - (2 * lado_a * lado_b * cos(angulo_rad));
    
    % Se extrae la raíz cuadrada para obtener la magnitud real
    lado_resultante = sqrt(lado_cuadrado);
end