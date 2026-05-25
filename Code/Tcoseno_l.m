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