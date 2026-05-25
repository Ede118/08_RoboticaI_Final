function [T] = A_DH(theta, d, a, alpha)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    theta
    d
    a
    alpha
end

arguments (Output)
    T
end

T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1];

end
