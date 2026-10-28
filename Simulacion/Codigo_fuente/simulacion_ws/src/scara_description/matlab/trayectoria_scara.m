clc;
clear;
close all;

%% ================================================================
%  SCARA R-R-P: trayectoria cartesiana recta + cinematica inversa
%  Dimensiones tomadas del URDF actual.
%  Limite real de J2 medido antes del choque: +/-1.750 rad.
%% ================================================================

%% GEOMETRIA Y LIMITES
L1 = 0.15000;       % [m] J1 -> J2
L2 = 0.14617;       % [m] J2 -> eje prismático J3
z0 = 0.303 + 0.083 + 0.0345;  % [m] altura del origen de J3 con q3=0

q1_min = -pi;
q1_max =  pi;
q2_lim =  1.750;    % [rad] +/-100.27 deg, limite medido
q2_min = -q2_lim;
q2_max =  q2_lim;
q3_min = -0.084;    % [m]
q3_max =  0.084;    % [m]

%% TIEMPO
T  = 5;             % [s] tiempo total
% Para la entrega puedes usar dt=1 si el profesor pide un punto por segundo.
% Para Gazebo se recomienda una trayectoria mas suave:
dt = 0.10;          % [s]
t = (0:dt:T)';

%% BUSCAR UN DESPLAZAMIENTO RECTO MUY LARGO Y ALCANZABLE
% Con q2 limitado, el workspace XY es aproximadamente un anillo:
r_max = L1 + L2;
r_min = sqrt(L1^2 + L2^2 + 2*L1*L2*cos(q2_lim));

% Chord tangente al radio interior: evita atravesar zona no alcanzable.
a = sqrt(r_max^2 - r_min^2);

% Se usa tambien todo el recorrido vertical de J3.
P0 = [-a, r_min, z0 + q3_min];
Pf = [ a, r_min, z0 + q3_max];

%% DISTANCIA Y VELOCIDAD CARTESIANA CONSTANTE
distancia = norm(Pf - P0);
velocidad = distancia / T;

fprintf('P0 = [%.4f %.4f %.4f] m\n', P0);
fprintf('Pf = [%.4f %.4f %.4f] m\n', Pf);
fprintf('Distancia = %.4f m\n', distancia);
fprintf('Velocidad cartesiana constante = %.4f m/s\n', velocidad);
fprintf('r_min = %.4f m, r_max = %.4f m\n', r_min, r_max);

%% INTERPOLACION LINEAL CARTESIANA
lambda = t / T;
P = P0 + lambda .* (Pf - P0);
X = P(:,1); Y = P(:,2); Z = P(:,3);

%% CINEMATICA INVERSA SCARA
N = numel(t);
q1 = zeros(N,1);
q2 = zeros(N,1);
q3 = zeros(N,1);

for k = 1:N
    x = X(k); y = Y(k); z = Z(k);

    c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
    if abs(c2) > 1 + 1e-9
        error('Punto %d fuera del workspace. c2=%.6f', k, c2);
    end
    c2 = max(-1,min(1,c2));

    % Rama codo positivo. Mantiene q2 entre 0 y q2_lim en esta trayectoria.
    q2(k) = acos(c2);
    q1(k) = atan2(y,x) - atan2(L2*sin(q2(k)), L1 + L2*cos(q2(k)));
    q1(k) = atan2(sin(q1(k)), cos(q1(k)));  % envolver a [-pi,pi]
    q3(k) = z - z0;
end

%% VALIDACION DE LIMITES
if any(q1 < q1_min-1e-6 | q1 > q1_max+1e-6)
    error('La trayectoria viola el limite de J1');
end
if any(q2 < q2_min-1e-6 | q2 > q2_max+1e-6)
    error('La trayectoria viola el limite de J2');
end
if any(q3 < q3_min-1e-6 | q3 > q3_max+1e-6)
    error('La trayectoria viola el limite de J3');
end

%% VELOCIDADES ARTICULARES (DERIVADA NUMERICA)
dq1 = gradient(q1, dt);
dq2 = gradient(q2, dt);
dq3 = gradient(q3, dt);

%% TABLA Y CSV PARA ROS 2 / GAZEBO
Trayectoria = table(t, X, Y, Z, q1, q2, q3, dq1, dq2, dq3, ...
    'VariableNames', {'Tiempo_s','X_m','Y_m','Z_m', ...
    'q1_rad','q2_rad','q3_m','dq1_rad_s','dq2_rad_s','dq3_m_s'});

disp(Trayectoria);

% Guarda en la carpeta actual de MATLAB.
writetable(Trayectoria, 'trayectoria_scara.csv');
fprintf('\nCSV generado: trayectoria_scara.csv\n');

%% GRAFICA 3D
figure;
plot3(X,Y,Z,'-o');
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Trayectoria cartesiana recta del SCARA');
hold on;
plot3(P0(1),P0(2),P0(3),'o');
plot3(Pf(1),Pf(2),Pf(3),'o');
legend('Trayectoria','P_0','P_f','Location','best');

%% POSICIONES ARTICULARES
figure;
plot(t, rad2deg(q1), 'LineWidth',1.4); hold on;
plot(t, rad2deg(q2), 'LineWidth',1.4);
plot(t, q3*1000, 'LineWidth',1.4);
grid on;
xlabel('Tiempo [s]'); ylabel('Posicion articular');
legend('q_1 [deg]','q_2 [deg]','q_3 [mm]','Location','best');
title('Posiciones articulares');

%% VELOCIDADES ARTICULARES
figure;
plot(t, rad2deg(dq1), 'LineWidth',1.4); hold on;
plot(t, rad2deg(dq2), 'LineWidth',1.4);
plot(t, dq3*1000, 'LineWidth',1.4);
grid on;
xlabel('Tiempo [s]'); ylabel('Velocidad articular');
legend('dq_1 [deg/s]','dq_2 [deg/s]','dq_3 [mm/s]','Location','best');
title('Velocidades articulares');

%% NOTA SOBRE TORQUE
% El torque NO sale de la cinematica inversa. Para torque se necesita
% dinamica (masas, inercias, gravedad) o leer el esfuerzo de Gazebo.
% Las masas/inercias del URDF actual son aproximadas; por tanto, cualquier
% torque de Gazebo debe presentarse como estimacion de simulacion hasta
% reemplazarlas por propiedades de masa reales de SolidWorks.
