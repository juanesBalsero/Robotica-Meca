1; % Declaración Octave

clc; clear; close all;

% FUNCIONES AUXILIARES

function q_target_cont = desentranar_angulos(q_ref, q_target)
    % Ajustar q_target para tomar la menor distancia angular respecto a q_ref
    q_target_cont = q_target;
    for k = 1:length(q_target)
        diff = q_target(k) - q_ref(k);
        diff_wrapped = atan2(sin(diff), cos(diff));
        q_target_cont(k) = q_ref(k) + diff_wrapped;
    end
end

function [theta1, theta2, theta3] = cinematica_inversa_3dof(P, L1, L2, L3)
    px = P(1);
    py = P(2);
    pz = P(3);

    theta1 = atan2(py, px);

    cos_theta3 = (px^2 + py^2 + (pz - L1)^2 - L2^2 - L3^2) / (2 * L2 * L3);

    if abs(cos_theta3) > 1
        error('El punto ingresado está fuera del alcance del robot.');
    end

    sin_theta3 = -sqrt(1 - cos_theta3^2); % Codo arriba
    theta3 = atan2(sin_theta3, cos_theta3);

    alpha = atan2(pz - L1, sqrt(px^2 + py^2));
    beta  = atan2(L3 * sin_theta3, L2 + L3 * cos_theta3);
    theta2 = alpha - beta;

    if nargout <= 1
        theta1 = [theta1; theta2; theta3];
    end
end

function Jp = evaluar_jacobiano_3dof(q, L1, L2, L3)
    A1 = DH_local(q(1), L1, pi/2, 0);
    A2 = DH_local(q(2), 0,  0,    L2);
    A3 = DH_local(q(3), 0,  0,    L3);

    T1 = A1;
    T2 = T1 * A2;
    T3 = T2 * A3;

    P0 = [0; 0; 0];
    Z0 = [0; 0; 1];

    P1 = T1(1:3, 4); Z1 = T1(1:3, 3);
    P2 = T2(1:3, 4); Z2 = T2(1:3, 3);
    Pe = T3(1:3, 4);

    Jp = zeros(3, 3);
    Jp(:, 1) = cross(Z0, (Pe - P0));
    Jp(:, 2) = cross(Z1, (Pe - P1));
    Jp(:, 3) = cross(Z2, (Pe - P2));
end

function T = DH_local(th, d, al, a)
    T = [ ...
        cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th); ...
        sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th); ...
        0,        sin(al),          cos(al),          d; ...
        0,        0,                0,                1 ...
    ];
end

% SCRIPT PRINCIPAL

% CONSTANTES DEL ROBOT
L1 = 0.2000;
L2 = 0.1506;
L3 = 0.18464;

% DEFINICIÓN DE LOS PUNTOS
P_Home = [-0.2;  0.00000; 0.05];
P3     = [0.00000;  0.24735; 0.21800];
P4     = [0.00000;  0.24735; 0.16800];
P_Aux  = [-0.3;   0.00000; 0.16800];
P_Aux1 = [-0.2;  -0.2;     0.16800];
P_Aux2 = [-0.2;   0.2;     0.16800];
P1     = [0.00000; -0.28735; 0.16800];
P2     = [0.00000; -0.28735; 0.11800];

% CINEMÁTICA INVERSA EN CADA PUNTO
Q_Home  = cinematica_inversa_3dof(P_Home, L1, L2, L3);
Q_pAux  = desentranar_angulos(Q_Home, cinematica_inversa_3dof(P_Aux, L1, L2, L3));
Q_pAux1 = desentranar_angulos(Q_pAux, cinematica_inversa_3dof(P_Aux1, L1, L2, L3));
Q_p1    = desentranar_angulos(Q_pAux1, cinematica_inversa_3dof(P1, L1, L2, L3));
Q_p2    = desentranar_angulos(Q_p1, cinematica_inversa_3dof(P2, L1, L2, L3));

Q_pAux2 = desentranar_angulos(Q_pAux, cinematica_inversa_3dof(P_Aux2, L1, L2, L3));
Q_p3    = desentranar_angulos(Q_pAux2, cinematica_inversa_3dof(P3, L1, L2, L3));
Q_p4    = desentranar_angulos(Q_p3, cinematica_inversa_3dof(P4, L1, L2, L3));

% CONFIGURACIÓN DE TRAYECTORIAS
N = 300;
w_pico_max = 0.2;

secuencia = {
    Q_Home,   Q_pAux,  P_Home,   P_Aux;   % T1
    Q_pAux,  Q_pAux1,  P_Aux,  P_Aux1;   % T2
    Q_pAux1,    Q_p1,  P_Aux1,     P1;   % T3
    Q_p1,       Q_p2,     P1,      P2;   % T4  (Llegada a P2)
    Q_p2,       Q_p1,     P2,      P1;   % T5
    Q_p1,    Q_pAux1,     P1,  P_Aux1;   % T6
    Q_pAux1,  Q_pAux,  P_Aux1,  P_Aux;   % T7
    Q_pAux,  Q_pAux2,  P_Aux,  P_Aux2;   % T8
    Q_pAux2,    Q_p3,  P_Aux2,     P3;   % T9
    Q_p3,       Q_p4,     P3,      P4;   % T10 (Llegada a P4)
    Q_p4,       Q_p3,     P4,      P3;   % T11
    Q_p3,    Q_pAux2,     P3,  P_Aux2;   % T12
    Q_pAux2,  Q_pAux,  P_Aux2,  P_Aux;   % T13
    Q_pAux,   Q_Home,  P_Aux,  P_Home    % T14
};

num_traj = size(secuencia, 1);
cfg = struct();

% Cálculo de parámetros de trayectoria continua
for i = 1:num_traj
    cfg(i).q_ini = secuencia{i, 1};
    cfg(i).q_fin = desentranar_angulos(cfg(i).q_ini, secuencia{i, 2});
    cfg(i).P_ini = secuencia{i, 3};
    cfg(i).P_fin = secuencia{i, 4};

    vec_dir = cfg(i).P_fin - cfg(i).P_ini;
    distancia = norm(vec_dir);

    if distancia > 0
        cfg(i).u = vec_dir / distancia;
    else
        cfg(i).u = [0; 0; 0];
    end

    delta_q_max = max(abs(cfg(i).q_fin - cfg(i).q_ini));
    T_seg = (1.25 * delta_q_max) / w_pico_max;

    if T_seg < 1.0
        T_seg = 1.0; % Tiempo mínimo de seguridad
    end

    cfg(i).t_final = T_seg;
    cfg(i).v_pico = 1.25 * (distancia / T_seg);
end

% INTEGRACIÓN CARTESIANA
q_trayectorias_total = cell(1, num_traj);
qtot = cell(1, num_traj);
q_actual = Q_Home;

for i = 1:num_traj
    P_ini    = cfg(i).P_ini;
    P_fin    = cfg(i).P_fin;
    u        = cfg(i).u;
    v_pico   = cfg(i).v_pico;
    t_final  = cfg(i).t_final;

    dt = t_final / N;
    P_dot = zeros(3, N);

    n_rampa = round(0.2 * N);
    x = v_pico / n_rampa;

    for h = 1:n_rampa
        P_dot(:, h) = h * x * u;
    end
    for h = (n_rampa + 1):(N - n_rampa)
        P_dot(:, h) = v_pico * u;
    end
    for h = (N - n_rampa + 1):N
        P_dot(:, h) = (v_pico - (h - (N - n_rampa)) * x) * u;
    end

    q_trayectoria = zeros(3, N);
    q_trayectoria(:, 1) = q_actual;
    qdot = zeros(3, N);

    for k = 1:(N - 1)
        Jp = evaluar_jacobiano_3dof(q_actual, L1, L2, L3);
        q_dot = pinv(Jp) * P_dot(:, k);
        qdot(:, k) = q_dot;
        q_actual = q_actual + q_dot * dt;
        q_trayectoria(:, k + 1) = q_actual;
    end
    qdot(:, N) = qdot(:, N-1);

    qtot{i} = qdot;
    q_trayectorias_total{i} = q_trayectoria;
end

% --- CONCATENAR DATOS ---
q_total = [];
qdotTotal = [];
t_total = [];
t_acumulado = 0;
t_cambios = zeros(1, num_traj);

for i = 1:num_traj
    t_final_traj = cfg(i).t_final;
    t_local = linspace(t_acumulado, t_acumulado + t_final_traj, N);

    t_total = [t_total, t_local];
    q_total = [q_total, q_trayectorias_total{i}];
    qdotTotal = [qdotTotal, qtot{i}];
    t_acumulado = t_acumulado + t_final_traj;
    t_cambios(i) = t_acumulado;
end

% --- GRÁFICA DE POSICIONES ARTICULARES ---
figure('Name','Cinemática Inversa 3 DOF - Posiciones Continuas','Color','w');
hold on; grid on;
colores = lines(3);
nombres = {'q_1','q_2','q_3'};

for j = 1:3
    plot(t_total, rad2deg(q_total(j,:)), 'LineWidth', 1.5, 'Color', colores(j,:), 'DisplayName', nombres{j});
end

title('Evolución de las 3 articulaciones (Octave)','FontSize',12);
xlabel('Tiempo (s)','FontSize',11);
ylabel('Posición (grados)','FontSize',11);
legend('Location','bestoutside','FontSize',10);
xlim([0 t_acumulado]);

y_lim = ylim;

for i = 1:num_traj
    plot([t_cambios(i) t_cambios(i)], y_lim, 'k--', 'LineWidth', 1, 'HandleVisibility','off');

    if i == 1
        x_text = t_cambios(i) / 2;
    else
        x_text = (t_cambios(i) + t_cambios(i-1)) / 2;
    end

    text(x_text, y_lim(2), sprintf('T%d',i), ...
        'VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',8,'Color','k');
end
hold off;

% --- GRÁFICA DE VELOCIDADES ARTICULARES ---
figure('Name','Cinemática Inversa 3 DOF - Velocidades','Color','w');
hold on; grid on;
nombres_v = {'qdot_1','qdot_2','qdot_3'};

for j = 1:3
    plot(t_total, qdotTotal(j,:), 'LineWidth', 1.5, 'Color', colores(j,:), 'DisplayName', nombres_v{j});
end

title('Evolución de Velocidades Articulares','FontSize',12);
xlabel('Tiempo (s)','FontSize',11);
ylabel('Velocidad (rad/s)','FontSize',11);
legend('Location','bestoutside','FontSize',10);
xlim([0 t_acumulado]);

y_lim = ylim;

for i = 1:num_traj
    plot([t_cambios(i) t_cambios(i)], y_lim, 'k--', 'LineWidth', 1, 'HandleVisibility','off');

    if i == 1
        x_text = t_cambios(i) / 2;
    else
        x_text = (t_cambios(i) + t_cambios(i-1)) / 2;
    end

    text(x_text, y_lim(2), sprintf('T%d',i), ...
        'VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',8,'Color','k');
end
hold off;

% --- CINEMÁTICA DIRECTA PARA GRÁFICA 3D ---
num_puntos = size(q_total, 2);
Pe_total = zeros(3, num_puntos);

for k = 1:num_puntos
    q_k = q_total(:, k);

    A1 = DH_local(q_k(1), L1, pi/2, 0);
    A2 = DH_local(q_k(2), 0,  0,    L2);
    A3 = DH_local(q_k(3), 0,  0,    L3);

    T03 = A1 * A2 * A3;
    Pe_total(:, k) = T03(1:3, 4);
end

figure('Name', 'Trayectoria 3D del Efector Final', 'Color', 'w');
hold on; grid on;

plot3(Pe_total(1,:), Pe_total(2,:), Pe_total(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trayectoria Efector');

P_obj = [P_Home, P1, P2, P_Aux, P_Aux1, P_Aux2, P3, P4];
etiquetas = {'Home', 'P1', 'P2', 'Aux', 'Aux1', 'Aux2', 'P3', 'P4'};

scatter3(P_obj(1,:), P_obj(2,:), P_obj(3,:), 60, 'ro', 'filled', 'DisplayName', 'Puntos de Referencia');

for i = 1:length(etiquetas)
    text(P_obj(1,i), P_obj(2,i), P_obj(3,i) + 0.008, etiquetas{i}, ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
end

title('Trayectoria Cartesiana del Efector Final (3 DOF)', 'FontSize', 12);
xlabel('X (m)', 'FontWeight', 'bold');
ylabel('Y (m)', 'FontWeight', 'bold');
zlabel('Z (m)', 'FontWeight', 'bold');

axis equal;
view(3);
legend('Location', 'best');
hold off;

% --- EXPORTAR CSV ---
puntos_por_traj = N;
gripper_state = zeros(1, num_puntos);

v_abierto = 0.0;   % Expandido (0.0 m)
v_cerrado = 0.15;  % Retraído (0.15 m)

idx_P2 = 4 * puntos_por_traj;
idx_P4 = 10 * puntos_por_traj;
n_rampa_g = round(0.15 * N);

gripper_state(1 : idx_P2 - n_rampa_g) = v_cerrado;
gripper_state(idx_P2 - n_rampa_g + 1 : idx_P2) = linspace(v_cerrado, v_abierto, n_rampa_g);
gripper_state(idx_P2 + 1 : idx_P4 - n_rampa_g) = v_abierto;
gripper_state(idx_P4 - n_rampa_g + 1 : idx_P4) = linspace(v_abierto, v_cerrado, n_rampa_g);
gripper_state(idx_P4 + 1 : end) = v_cerrado;

datos_csv = [t_total', q_total', gripper_state'];
dlmwrite('trayectoria_robot.csv', datos_csv, 'precision', '%.6f');
disp('CSV generado correctamente en Octave.');
