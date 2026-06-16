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