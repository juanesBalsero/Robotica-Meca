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
    px = P(1); py = P(2); pz = P(3);

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

    T1 = A1; T2 = T1 * A2; T3 = T2 * A3;

    P0 = [0; 0; 0]; Z0 = [0; 0; 1];
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

% DEFINICIÓN DE LOS PUNTOS CARTESIANOS
P_Home = [-0.2;   0.00000; 0.05];
P3     = [0.00000;  0.24735; 0.21800];
P4     = [0.00000;  0.24735; 0.16800];
P_Aux  = [-0.3;   0.00000; 0.16800];
P_Aux1 = [-0.2;  -0.2;     0.16800];
P_Aux2 = [-0.2;   0.2;     0.16800];
P1     = [0.00000; -0.28735; 0.16800];
P2     = [0.00000; -0.28735; 0.11800];

% CINEMÁTICA INVERSA CONTINUA EN CADA PUNTO
Q_Home  = cinematica_inversa_3dof(P_Home, L1, L2, L3);
Q_pAux  = desentranar_angulos(Q_Home,  cinematica_inversa_3dof(P_Aux,  L1, L2, L3));
Q_pAux1 = desentranar_angulos(Q_pAux,  cinematica_inversa_3dof(P_Aux1, L1, L2, L3));
Q_p1    = desentranar_angulos(Q_pAux1, cinematica_inversa_3dof(P1,     L1, L2, L3));
Q_p2    = desentranar_angulos(Q_p1,    cinematica_inversa_3dof(P2,     L1, L2, L3));

Q_pAux2 = desentranar_angulos(Q_pAux,  cinematica_inversa_3dof(P_Aux2, L1, L2, L3));
Q_p3    = desentranar_angulos(Q_pAux2, cinematica_inversa_3dof(P3,     L1, L2, L3));
Q_p4    = desentranar_angulos(Q_p3,    cinematica_inversa_3dof(P4,     L1, L2, L3));

% SECUENCIA COMPLETA DE TRAYECTORIAS (14 TRAMOS)
secuencia = {
    Q_Home,   Q_pAux,  P_Home,  P_Aux;   % T1
    Q_pAux,  Q_pAux1,  P_Aux,   P_Aux1;  % T2
    Q_pAux1,    Q_p1,  P_Aux1,  P1;      % T3
    Q_p1,       Q_p2,  P1,      P2;      % T4  (Llegada a P2)
    Q_p2,       Q_p1,  P2,      P1;      % T5
    Q_p1,    Q_pAux1,  P1,      P_Aux1;  % T6
    Q_pAux1,  Q_pAux,  P_Aux1,  P_Aux;   % T7
    Q_pAux,  Q_pAux2,  P_Aux,   P_Aux2;  % T8
    Q_pAux2,    Q_p3,  P_Aux2,  P3;      % T9
    Q_p3,       Q_p4,  P3,      P4;      % T10 (Llegada a P4)
    Q_p4,       Q_p3,  P4,      P3;      % T11
    Q_p3,    Q_pAux2,  P3,      P_Aux2;  % T12
    Q_pAux2,  Q_pAux,  P_Aux2,  P_Aux;   % T13
    Q_pAux,   Q_Home,  P_Aux,   P_Home   % T14
};

num_traj = size(secuencia, 1);
N = 300; % Puntos por trayectoria
w_pico_max = 0.2; % Límite máximo

xd_total = [];
xd_punto_total = [];
q_total = [];
qdot_total = [];
t_total = [];
t_acumulado = 0;
t_cambios = zeros(1, num_traj);

q_ref_prev = Q_Home;

% Planeación en el espacio de trabajo
for i = 1:num_traj
    q_ini = desentranar_angulos(q_ref_prev, secuencia{i, 1});
    q_fin = desentranar_angulos(q_ini, secuencia{i, 2});
    P_ini = secuencia{i, 3};
    P_fin = secuencia{i, 4};

    vec_dir = P_fin - P_ini;
    S_total = norm(vec_dir);

    if S_total > 0
        u = vec_dir / S_total;
    else
        u = [0; 0; 0];
    end

    delta_q_max = max(abs(q_fin - q_ini));
    T_seg = max(1.0, (1.65 * delta_q_max) / w_pico_max);

    % Bucle de ajuste de tiempo para asegurar w_pico_max <= 0.2 rad/s
    mantenimiento = true;
    while mantenimiento
        s_dot_c = 1.25 * (S_total / T_seg);
        if s_dot_c >= (2 * S_total / T_seg)
            s_dot_c = 1.95 * (S_total / T_seg);
        end

        % Ecuaciones de Siciliano
        tc = (0 - S_total + s_dot_c * T_seg) / s_dot_c;
        s_ddot = (s_dot_c^2) / (s_dot_c * T_seg - S_total);

        t_local = linspace(0, T_seg, N);
        s = zeros(1, N);
        s_dot = zeros(1, N);

        for k = 1:N
            tk = t_local(k);
            if tk <= tc
                s(k) = 0.5 * s_ddot * tk^2;
                s_dot(k) = s_ddot * tk;
            elseif tk <= (T_seg - tc)
                s(k) = s_ddot * tc * (tk - tc/2);
                s_dot(k) = s_dot_c;
            else
                s(k) = S_total - 0.5 * s_ddot * (T_seg - tk)^2;
                s_dot(k) = s_ddot * (T_seg - tk);
            end
        end

        xd_k = zeros(3, N);
        xd_dot_k = zeros(3, N);
        q_k = zeros(3, N);
        qdot_k = zeros(3, N);

        for k = 1:N
            xd_k(:, k) = P_ini + s(k) * u;
            xd_dot_k(:, k) = s_dot(k) * u;

            q_raw = cinematica_inversa_3dof(xd_k(:, k), L1, L2, L3);
            if k == 1
                q_k(:, k) = desentranar_angulos(q_ref_prev, q_raw);
            else
                q_k(:, k) = desentranar_angulos(q_k(:, k-1), q_raw);
            end

            Jp = evaluar_jacobiano_3dof(q_k(:, k), L1, L2, L3);
            qdot_k(:, k) = pinv(Jp) * xd_dot_k(:, k);
        end

        max_w = max(max(abs(qdot_k)));
        if max_w <= w_pico_max
            mantenimiento = false;
        else
            % Escalar T_seg para garantizar que la velocidad máxima respete el límite
            T_seg = T_seg * (max_w / (0.95 * w_pico_max));
        end
    end

    q_ref_prev = q_k(:, end);

    t_abs = linspace(t_acumulado, t_acumulado + T_seg, N);
    t_acumulado = t_acumulado + T_seg;
    t_cambios(i) = t_acumulado;

    t_total = [t_total, t_abs];
    xd_total = [xd_total, xd_k];
    xd_punto_total = [xd_punto_total, xd_dot_k];
    q_total = [q_total, q_k];
    qdot_total = [qdot_total, qdot_k];
end

% ESTADO DEL GRIPPER Y GENERACIÓN DE CSV
num_puntos = size(xd_total, 2);
gripper_state = zeros(1, num_puntos);

v_abierto = 0.0;   % Expandido (0.0 m)
v_cerrado = 0.15;  % Retraído (0.15 m)

idx_P2 = 4 * N;   % Fin de T4 (Llegada a P2)
idx_P4 = 10 * N;  % Fin de T10 (Llegada a P4)
n_rampa_g = round(0.15 * N);

gripper_state(1 : idx_P2 - n_rampa_g) = v_cerrado;
gripper_state(idx_P2 - n_rampa_g + 1 : idx_P2) = linspace(v_cerrado, v_abierto, n_rampa_g);
gripper_state(idx_P2 + 1 : idx_P4 - n_rampa_g) = v_abierto;
gripper_state(idx_P4 - n_rampa_g + 1 : idx_P4) = linspace(v_abierto, v_cerrado, n_rampa_g);
gripper_state(idx_P4 + 1 : end) = v_cerrado;

% Exportación del CSV: Cols 1..3: xd | Col 4: Gripper | Cols 5..7: xd_punto
datos_csv = [xd_total', gripper_state', xd_punto_total'];
dlmwrite('trayectoria_robot.csv', datos_csv, 'precision', '%.6f');
disp('CSV generado correctamente: trayectoria_robot.csv');

% GRAFICACIÓN DE RESULTADOS

% POSICIÓN Y VELOCIDAD ARTICULAR
figure('Name','Evolución Articular Continua','Color','w');
subplot(2,1,1);
hold on; grid on;
colores = lines(3);
for j = 1:3
    plot(t_total, rad2deg(q_total(j,:)), 'LineWidth', 1.5, 'Color', colores(j,:));
end
title('Posición Articular q(t) [grados]');
ylabel('Grados'); legend('q_1', 'q_2', 'q_3', 'Location', 'bestoutside');

subplot(2,1,2);
hold on; grid on;
for j = 1:3
    plot(t_total, qdot_total(j,:), 'LineWidth', 1.5, 'Color', colores(j,:));
end
yline(w_pico_max, 'r--', 'Límite +w_{pico}');
yline(-w_pico_max, 'r--', 'Límite -w_{pico}');
title('Velocidad Articular dq(t)/dt [rad/s]');
xlabel('Tiempo [s]'); ylabel('rad/s'); legend('\dot{q}_1', '\dot{q}_2', '\dot{q}_3', 'Location', 'bestoutside');

% POSICIÓN Y VELOCIDAD CARTESIANA DESEADA (xd y xd_punto)
figure('Name','Perfiles Cartesianos Referencia xd y xd_punto','Color','w');
subplot(2,1,1);
plot(t_total, xd_total', 'LineWidth', 1.5); grid on;
title('Posición Cartesiana Deseada x_d(t) [m]');
ylabel('Posición [m]'); legend('X_d', 'Y_d', 'Z_d', 'Location', 'bestoutside');

subplot(2,1,2);
plot(t_total, xd_punto_total', 'LineWidth', 1.5); grid on;
title('Velocidad Cartesiana Deseada \dot{x}_d(t) [m/s]');
xlabel('Tiempo [s]'); ylabel('Velocidad [m/s]'); legend('\dot{X}_d', '\dot{Y}_d', '\dot{Z}_d', 'Location', 'bestoutside');

% TRAYECTORIA CARTESIANA 3D
figure('Name', 'Trayectoria Cartesiana 3D Efector Final', 'Color', 'w');
hold on; grid on;
plot3(xd_total(1,:), xd_total(2,:), xd_total(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'x_d(t) Siciliano');

P_obj = [P_Home, P1, P2, P_Aux, P_Aux1, P_Aux2, P3, P4];
etiquetas = {'Home', 'P1', 'P2', 'Aux', 'Aux1', 'Aux2', 'P3', 'P4'};

scatter3(P_obj(1,:), P_obj(2,:), P_obj(3,:), 60, 'ro', 'filled', 'DisplayName', 'Puntos Referencia');

for i = 1:length(etiquetas)
    text(P_obj(1,i), P_obj(2,i), P_obj(3,i) + 0.008, etiquetas{i}, 'FontSize', 10, 'FontWeight', 'bold');
end

title('Trayectoria Cartesiana 3D del Efector Final (14 Tramos)');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
axis equal; view(3); legend('Location', 'best'); hold off;
