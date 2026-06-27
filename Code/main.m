%% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

fprintf('La trayectoria original es OUT.\nLa trayectoria IN es solo de prueba (requiere revision).\n');

fprintf('¿Trayectoria Interna o Externa?\n');
trayectoria = input('[IN/OUT] ', 's');

cartesianos = strcmpi(input('Guardar graficos cartesianos: [y/n] ', 's'), 'y');
articulares = strcmpi(input('Guardar graficos articulares: [y/n] ', 's'), 'y');
jacobiano = strcmpi(input('Guardar grafico de Jacobiano: [y/n] ', 's'), 'y');
grabar = strcmpi(input('Grabar simulacion: [y/n] ', 's'), 'y');

if strcmp(trayectoria, 'IN')
    TrayectoriaInterior(cartesianos, articulares, jacobiano, grabar);
elseif strcmp(trayectoria, 'OUT')
    TrayectoriaExterior(cartesianos, articulares, jacobiano, grabar);
else 
    fprintf('Trayectoria no valida\n');
end