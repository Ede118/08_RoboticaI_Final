%% Mapeo de la Submatriz J11 con Intersección Z=0
a = 9360000; b = 24336000; c = 79872000;
alpha = b/a; beta = c/a;

f = @(q2,q3) -alpha*sin(deg2rad(q2)).*cos(deg2rad(q3)).^2 - alpha*cos(deg2rad(q2)).*sin(deg2rad(q3)).*cos(deg2rad(q3)).^2 + alpha*sin(deg2rad(q2)) - sin(deg2rad(q3)) - beta*cos(deg2rad(q2)).*sin(deg2rad(q3));

figure(2)

% Subplot 1: Superficie 3D automática
subplot(1, 2, 1)
fsurf(f, [-360, 360, -360, 360]) 
title('Superficie 3D con fsurf')
xlabel('q2'); ylabel('q3'); zlabel('det')
grid on

% Subplot 2: Mapa de contorno automático
subplot(1, 2, 2)
fcontour(f, [-360, 360, -360, 360], 'Fill', 'on') 
hold on  

fcontour(f, [-360, 360, -360, 360], 'LevelList', 0, 'LineColor', 'r', 'LineWidth', 2.5)

title('Líneas de Singularidad Física (det = 0)')
xlabel('q2'); ylabel('q3')
colorbar
hold off