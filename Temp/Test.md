# Code Test

# Seeds Initialization
```matlab
robot


N = 1001;
n = (0:1000)';


QSolution = zeros(N, 6);


% Vectorize the solution vector Q
QSolution(:, 1) = -pi + 2*pi*rand(N, 1);


QSolution(:, 2) = -pi;
QSolution(:, 2) = QSolution(:, 2) + (2*pi/1000) * (0:N-1)'; 


CSolution = Robot.fkine(QSolution);


if isa(CSolution,'SE3'), CSolution = CSolution.T; end


[ROT, p] = tr2rt(CSolution);
angles = tr2rpy(CSolution, 'xyz');
x = [p, angles];


```

# Solution $q_1$ 
```matlab
% 1001 Objects SE3
Target = SE3(x(:, 1:3)) * SE3.rpy(x(:, 4:6), 'zyx');


hat = Robot.offset';
Robot.offset = zeros(6, 1);


T_tool = Robot.tool;


d6           = Robot.links(6).d; 
T06          = repmat(SE3(), N, 1);
p_w          = zeros(N, 3);
q1           = zeros(N, 1);
q2           = zeros(N, 2);
q3           = zeros(N, 2);


a2 = Robot.links(2).a; 
a3 = Robot.links(3).a; 
d4 = Robot.links(4).d; 


for k = 1:N
    T06(k) = Target(k) / T_tool; 
    
    % Kinematic decoupling
    % p_w = p - d6 * z
    pos_tcp = T06(k).t;
    z_tcp  = T06(k).a;
    
    p_w(k, :) = (pos_tcp - z_tcp * d6)';
    
    if norm(p_w(k, 1:2)) < 1e-6
        disp('Advertencia: Punto sobre el eje Z0 (Singularidad)')
    end
    
    q1(k) = atan2(p_w(k, 2), p_w(k, 1));


end


error = 100*((QSolution(:, 1) - q1)/QSolution(:, 1));


figure('Name', "Error q1"); 
plot(n(1:900), error(1:900), 'LineWidth', 1);
title('Error Relativo $q_1$');
legend('error', 'Location', 'northeastoutside');
grid on;grid minor;


```

# Solution $q_2 ,q_3$ 
```matlab
for k = 1:N
    
    T01 = Robot.links(1).A(q1(k)); 
    R01 = T01.R;
    
    p14_local = R01' * (p_w(k, :)' - T01.t); 
    
    % plane distance r, height z
    r = sqrt(p14_local(1)^2 + p14_local(2)^2);
    z = p14_local(3); 
    
    D_sq = r^2 + z^2
    cos_q3 = (d_w_sq - a2^2 - a3^2) / (2 * a2 * a3);
    
    cos_q3 = max(min(cos_q3, 1), -1);
    
    % q3 (dos soluciones: codo arriba / codo abajo)
    q3(k, 1) = atan2(sqrt(1 - cos_q3^2), cos_q3);
    q3(k, 2) = atan2(-sqrt(1 - cos_q3^2), cos_q3);
    
    % 4. Cálculo de q2 (depende de q3)
    for i = 1:2
        q2(k, i) = atan2(z, r) - atan2(a3*sin(q3(k, i)), a2 + a3*cos(q3(k, i)));
    end
end


error = 100*((QSolution(:, 2) - q1)/QSolution(:, 2));


figure('Name', "Error q2"); 
plot(n, error, 'LineWidth', 1);
title('Error Relativo $q_2$');
legend('error', 'Location', 'northeastoutside');
grid on;grid minor;


```