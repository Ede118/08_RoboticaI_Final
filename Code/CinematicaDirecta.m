function [XSol] = CinematicaDirecta(q, R)
% Función de Cinemática Directa
% 
% Inputs:
%   - q [Nx6]   :      Array de valores articulares en [rad]
%   - R         :      Objeto Serial Link (Peter Corke)
% 
% Outputs:
%   - XSol [Nx6]:      Array de valores cartesianos resolviendo la C.D.
%                      [x, y, z, roll, pitch, yaw] en convención 'zyx'

arguments
    q (:,6) double
    R
end

N = size(q, 1);
XSol = zeros(N, 6);


offset = zeros(1, 6);
for i = 1:6
    offset(i) = R.links(i).offset;
end


for i = 1:N
    T = SE3();

    for k = 1:6

        theta = q(i, k) + offset(k);
        
        % Parámetros del eslabón k
        d_k = R.links(k).d;
        a_k = R.links(k).a;
        alpha_k = R.links(k).alpha;
        
        % Matriz de transformación Denavit-Hartenberg estándar para el eslabón k
        A_k = [ ...
            cos(theta), -sin(theta)*cos(alpha_k),  sin(theta)*sin(alpha_k), a_k*cos(theta);
            sin(theta),  cos(theta)*cos(alpha_k), -cos(theta)*sin(alpha_k), a_k*sin(theta);
            0,           sin(alpha_k),             cos(alpha_k),            d_k;
            0,           0,                        0,                       1 ...
        ];
        
        T = T * SE3(A_k);
    end
    
    T_total = R.base * T * R.tool;
    
    % Posición y orientación (Euler RPY en convención 'zyx')
    XSol(i, 1:3) = T_total.t';
    XSol(i, 4:6) = T_total.tr2rpy('zyx');
end

end
