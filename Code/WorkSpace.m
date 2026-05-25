%% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

%clear; clc; close all;
robot

%% Definicion del WorkSpace
x1lim = -2;
x2lim = 2;
y1lim = -2;
y2lim = 2;
z1lim = -0.1;
z2lim = 2;

WS = [x1lim x2lim y1lim y2lim z1lim z2lim];


%% Mostrar sistemas de referencia {S_i}
q = zeros(1, R.n);
L = 0.35;

figure('Color','w'); grid on; axis equal;
view(135,25);

% Opciones:
% - 'nowrist', ...
% -
% -

R.plot(q, ...
    'workspace', WS, ...
    'notiles', ...
    'scale', 0.75, ...
    'jointdiam', 1.5, ...
    'jointlen', 1, ...
    'linkcolor', [.2 .2 .2], ...
    'jointcolor', [1 .4 0]);
R.teach();
hold on;

colores = [0.000, 0.278, 0.671; 0.843, 0.000, 0.251; 0.314, 0.784, 0.471; 1.000, 0.749, 0.000; 0.502, 0.000, 0.502];

% Opcion de Graficar el robot con los sistemas de coordenada
if false
    % ----- Grafico de {S_i} -----
    T = R.base;
    for m = 1:R.n
        if (show_axis(m) == 1) && m == 1
            trplot(R.base, 'frame','0', 'length', L);
        end
        if (show_axis(m) == 1) && m > 1
            Ai = R.A(m-1, q);
            T  = T * Ai;
            if m <= length(show_axis) && show_axis(m) == 1
                trplot(T, 'frame', ...
                    num2str(m-1), ...
                    'length', L, ...
                    'width', 0.5,...
                    'thick', 10,...
                    'axis', [-2 2 -2 2 -0.2 2], ...
                    'rgb', ...
                    'arrow');
            end
        end
    end
    
    % (Optional) Mostrar el tool como {T}
    if true
        trplot(T * R.tool, 'frame','T', 'length', L);
        title('Frames seleccionados');
    end
end

%% Graficos del WorkSpace

