function [Q] = CinematicaInversa(R, vector_p)
    % CINEMATICAINVERSA Calcula las 8 soluciones articulares (q) para un robot de 6GDL.
    arguments (Input)
        R           % Estructura o modelo del robot
        vector_p    % Vector de pose deseada (6x1)
    end
    arguments (Output)
        Q           % Matriz de 6x8 con las configuraciones articulares
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
    
    % Centro de la muñeca
    p_w = T06(1:3, 4) - T06(1:3, 3) * R.d(6);
    
    Q = zeros(6,8);
    
    %% 1. Soluciones para q1 (Hombro)
    if (p_w(1) == 0) && (p_w(2) == 0)
        disp('Advertencia: Punto sobre el eje Z0 (Singularidad)')
    end
    
    R_xy = sqrt(p_w(1)^2 + p_w(2)^2);
    
    d_offset = R.d(3); % <-- AJUSTA esto a R.d(3) si tu offset está en el eslabón 3.
    
    if R_xy < abs(d_offset)
        disp('Error: Punto inalcanzable. Está dentro del cilindro de singularidad base.');
        q1_front = 0;
        q1_back = 0;
    else
        alfa = atan2(p_w(2), p_w(1));
        beta = asin(d_offset / R_xy);
        
        q1_front = alfa - beta;
        q1_back  = alfa + beta + pi;
    end
    
    Q(1, 1:4) = wrapToPi(q1_front);
    Q(1, 5:8) = wrapToPi(q1_back);

    
    %% 2. Soluciones para q2 y q3 (Codo)
    % Iteramos sobre las dos configuraciones principales de q1

    for config_hombro = 1:2
        if config_hombro == 1
            cols = 1:4;
            q1_actual = q1_front;
        else
            cols = 5:8;
            q1_actual = q1_back;
        end
        
        A01 = A_DH(q1_actual - R.offset(1), R.d(1), R.a(1), R.alpha(1));
        
        
        % PROYECCIÓN: Convertir p_w al sistema local del eslabón 1.
        % La operación "A01 \ vector" resuelve los problemas de signos y rotaciones.
        p_w_local = A01 \ [p_w; 1];
        
        x_loc = p_w_local(1);
        y_loc = p_w_local(2);
        
        % Distancia planar
        r = sqrt(x_loc^2 + y_loc^2);

        L1 = R.a(2);
        % Hipotenusa real de J3 a Muñeca
        L2 = sqrt(R.a(3)^2 + R.d(4)^2); 
        
        % Angulos calculados con Teorema del coseno
        beta = Tcoseno_a(L1, r, L2);
        gamma = Tcoseno_a(L1, L2, r);
        
        % Ángulo de elevación del objetivo desde el origen de J2
        theta = atan2(y_loc, x_loc);
        
        % Configuración Codo Arriba
        q2_up = theta + beta;
        q3_up = pi - gamma - off;
        
        % Configuración Codo Abajo
        q2_down = theta - beta;
        q3_down = gamma - pi - off; 
        
        % Asignación a la matriz de soluciones
        Q(2, cols(1:2)) = q2_up;
        Q(3, cols(1:2)) = q3_up;
        
        Q(2, cols(3:4)) = q2_down;
        Q(3, cols(3:4)) = q3_down;
    end
    
    %% 3. Soluciones para q4, q5 y q6 (Muñeca Esférica)
    for i = 1:8
        A01 = A_DH(Q(1,i), R.d(1), R.a(1), R.alpha(1));
        A12 = A_DH(Q(2,i), R.d(2), R.a(2), R.alpha(2));
        A23 = A_DH(Q(3,i), R.d(3), R.a(3), R.alpha(3));
        
        T03 = A01 * A12 * A23;
        R03 = T03(1:3, 1:3);
        
        % Extraer la rotación de la muñeca despejando T03
        Rot36 = R03' * T06(1:3, 1:3);
        
        Q(4,i) = atan2(Rot36(2,3), Rot36(1,3));
        
        % Soluciones alternativas para el "flip" de la muñeca
        if mod(i, 2) ~= 0
            Q(5,i) = atan2(sqrt(Rot36(1,3)^2 + Rot36(2,3)^2), Rot36(3,3));
        else
            Q(5,i) = atan2(-sqrt(Rot36(1,3)^2 + Rot36(2,3)^2), Rot36(3,3));
        end
        
        Q(6,i) = atan2(-Rot36(3,2), Rot36(3,1));
    end
    
    %% 4. Aplicación de Offsets y Limpieza Angular
    Q = wrapToPi(Q);
    
    if isfield(R, 'offset')
        robot_offset = R.offset;
    else
        robot_offset = zeros(1, 6);
    end
    
    for s = 1:6
        Q(s, :) = Q(s, :) - robot_offset(s) * ones(1, 8);
    end
    
    Q = wrapToPi(Q);
end

