function [T] = x2T(x)
    if size(x,1) ~= 6
        disp('Advertencia: x debe ser de 6 filas')
    end
    pos = x(1:3, :);
    Rot = rpy2r(x(4:6, :)', 'zyx'); 
    T = rt2tr(Rot, pos);
end