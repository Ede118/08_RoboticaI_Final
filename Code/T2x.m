function [x] = T2x(Tin)
%T2X undefined
%   undefined
arguments (Input)
    Tin
end

arguments (Output)
    x
end

if isobject(Tin), Tin = Tin.T; end

[Rot, p] = tr2rt(Tin);
angles = tr2rpy(Tin, 'xyz')';
x = [p; angles];

end