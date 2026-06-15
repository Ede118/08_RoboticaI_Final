%%
x = [...
    0;...               % Coordenada x
    0.970;...           % Coordenada y
    1.395;...           % Coordenada z
    0;...               % Roll
    deg2rad(-80);...    % Pitch
    deg2rad(-90)...     % Yaw
];


c = zeros(6,1)';

offset = deg2rad([
    90; 
    75; 
    25; 
    0; 
    0; 
    0]);

base = transl(0,0,0);
d_tool = 0; 
tool = transl(0,0,d_tool);

T_tool = x2T(x);

T06 = T_tool / R.tool.T;

p_w = T06(1:3, 4) - T06(1:3,3)*R.d(6);

theta1 = atan2(p_w(2), p_w(1)) - R.offset(1);

