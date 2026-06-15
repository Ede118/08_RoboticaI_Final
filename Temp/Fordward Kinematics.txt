function [Tcartesiana] = CinematicaDirecta( ...
    sistema_inicial, ...
    sistema_final, ...
    matrizDH, ...
    offset, ...
    vectorq,...
    vectorx)

% CINEMATICADIRECTA undefined
%   undefined
%
%

arguments (Input)
    sistema_inicial
    sistema_final
    matrizDH
    offset
    vectorq
    vectorx
end

arguments (Output)
    Tcartesiana
end

if size(matrizDH,2) == 4
    d = matrizDH(:,1);
    theta = matrizDH(:,2);
    a = matrizDH(:,3);
    alpha = matrizDH(:,4);
else
    disp('Debe ingresarse Matriz DH (theta, d, a, alpha)')
end

if ~((0 < sistema_inicial) && (0 < sistema_inicial < 5))
    disp('Sistema inicial debe ser entre 0 y 5')
end
if ~((0 < sistema_final) && (0 < sistema_final < 5))
    disp('Sistema final debe ser entre 0 y 5')
end
if sistema_inicial >= sistema_final
    disp('Sistema final debe ser superior a inicial')
end


A = @(d,theta,a,alpha)...
    [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
    0, sin(alpha), cos(alpha), d,...
    0, 0, 0, 1];

vectorq = wrapToPi(vectorq);

theta = vectorq - offset;

Tcartesiana = vectorx;

for k = sistema_inicial:1:(sistema_final-sistema_inicial)
   Tcartesiana = A(d(k), theta(k), a(k), alpha(k))*vectorx;
end

end