clear all
close all

dt = 0.01;
coeff_M = 1.0;
coeff_g = 9.81;
coeff_Cw = 0.1;
coeff_Km = 0.1;
coeff_Jxx = 0.1;
coeff_Jyy = 0.1;
coeff_Jzz = 0.1;
coeff_Jw1 = 0.05;
coeff_Jw2 = 0.05;
coeff_Jw3 = 0.05;
%% Linearized Model (3D)
syms alpha beta gamma
% syms omega_hx omega_hy omega_hz
% syms omega_w1 omega_w2 omega_w3
syms Jxx Jyy Jzz
syms Jw1 Jw2 Jw3
syms M g Cw Km

F = [0 sin(gamma) / cos(beta) cos(gamma) / cos(beta);
        0   cos(gamma) -sin(beta);
        1 sin(gamma)*sin(beta)/cos(beta) cos(gamma)*sin(beta)/cos(beta)];

phi_0 = [pi/4, atan2(1, sqrt(2))]; % pitch, roll
F_0 = subs(F, [beta, gamma], [phi_0(1), phi_0(2)])

dgdphi = g * [0 cos(beta) 0;
                    0 sin(beta)*sin(gamma) -cos(beta)*cos(gamma);
                    0 sin(beta)*cos(gamma) cos(beta)*sin(gamma)];
dgdphi_0 = subs(dgdphi, [beta, gamma], [phi_0(1), phi_0(2)])

J = diag([Jxx, Jyy, Jzz]);
% inv(J)
Jw = diag([Jw1, Jw2, Jw3]);

A = [zeros(3), F_0, zeros(3);
    inv(J)*M*dgdphi_0, zeros(3), Cw*inv(J);
    -inv(J)*M*dgdphi_0, zeros(3), -Cw*(inv(J)+inv(Jw))]

B = [zeros(3);
        -inv(J)*Km;
        (inv(J)+inv(Jw))*Km]

A = subs(A, [M, g, Cw, Jxx, Jyy, Jzz, Jw1, Jw2, Jw3], [coeff_M, coeff_g, coeff_Cw, coeff_Jxx, coeff_Jyy, coeff_Jzz, coeff_Jw1, coeff_Jw2, coeff_Jw3])
B = subs(B, [Km, Jxx, Jyy, Jzz, Jw1, Jw2, Jw3], [coeff_Km, coeff_Jxx, coeff_Jyy, coeff_Jzz, coeff_Jw1, coeff_Jw2, coeff_Jw3])
%% simulation (3D)
A = [
    0         0         0         0    0.8165    1.1547         0         0         0;
    0         0         0         0    0.8165   -0.7071         0         0         0;
    0         0         0    1.0000    0.5774    0.8165         0         0         0;
    0   69.3672         0         0         0         0    1.0000         0         0;
    0   40.0492  -56.6381         0         0         0         0    1.0000         0;
    0   56.6381   40.0492         0         0         0         0         0    1.0000;
    0  -69.3672         0         0         0         0   -3.0000         0         0;
    0  -40.0492   56.6381         0         0         0         0   -3.0000         0;
    0  -56.6381  -40.0492         0         0         0         0         0   -3.0000]

B = [
    0     0     0;
    0     0     0;
     0     0     0;
    -1     0     0;
     0    -1     0;
     0     0    -1;
     3     0     0;
     0     3     0;
     0     0     3]

V = ctrb(A, B)
n = rank(V)
normV = V ./ sqrt(sum(V.^2));
V_independent = extractIndependentColumns(normV, n)
% rank(V_independent)

T8 = [1;1;1;0;1;0;1;0;0];
T = [V_independent, T8/norm(T8)]
rank(T)

bar_A = inv(T)*A*T
bar_B = inv(T)*B;

bar_A12 = bar_A(1:8, 1:8)
bar_B1 = bar_B(1:8, :)
rank(ctrb(bar_A12, bar_B1))
%%
Q = diag([10, 10, 10, 10, 10, 10, 10, 10]);
R = diag([1, 1, 1]);

[F, P, E] = lqrd(bar_A12, bar_B1, Q, R, dt);
F = -F

% simulation
tspan = [0:dt:10];
x0 = [0.2; 0.2; 0.2; 0.2; 0; 0; 0; 0; 0];

%% z coordinate
z0_ = inv(T)*x0;
z0 = z0_(1:8);
[t, z] = ode45(@(t, z) (bar_A12+bar_B1*F)*z, tspan, z0);

figure(1);
plot(t, z(:, 1), "LineWidth", 2);
grid on
hold on
plot(t, z(:, 2), "LineWidth", 2);
plot(t, z(:, 3), "LineWidth", 2);
plot(t, z(:, 4), "LineWidth", 2);
plot(t, z(:, 5), "LineWidth", 2);
plot(t, z(:, 6), "LineWidth", 2);
plot(t, z(:, 7), "LineWidth", 2);
plot(t, z(:, 8), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$z$", 'Interpreter', 'latex');
legend("$z_1$", "$z_2$", "$z_3$", "$z_4$", "$z_5$", "$z_6$", "$z_7$", "$z_8$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

%% z coordinate2
z0 = inv(T)*x0
[t, z] = ode45(@(t, z) (bar_A+bar_B*[F, zeros(3,1)])*z, tspan, z0);

figure(2);
plot(t, z(:, 1), "LineWidth", 2);
grid on
hold on
plot(t, z(:, 2), "LineWidth", 2);
plot(t, z(:, 3), "LineWidth", 2);
plot(t, z(:, 4), "LineWidth", 2);
plot(t, z(:, 5), "LineWidth", 2);
plot(t, z(:, 6), "LineWidth", 2);
plot(t, z(:, 7), "LineWidth", 2);
plot(t, z(:, 8), "LineWidth", 2);
plot(t, z(:, 9), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$z$", 'Interpreter', 'latex');
legend("$z_1$", "$z_2$", "$z_3$", "$z_4$", "$z_5$", "$z_6$", "$z_7$", "$z_8$", "$z_9$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

%% x coordinate
% x_f = [0; pi/4; atan2(1, sqrt(2)); 0; 0; 0; 0; 0; 0]; 

% [t, x] = ode45(@(t, x) T*(bar_A+bar_B*[F, zeros(3, 1)])*inv(T)*(x), tspan, x0);
[t, x] = ode45(@(t, x) (A+T*bar_B*[F, zeros(3, 1)]*inv(T))*(x), tspan, x0);

figure(3);
subplot(1, 3, 1);
plot(t, x(:, 1), "LineWidth", 2);
grid on
hold on
plot(t, x(:, 2), "LineWidth", 2);
plot(t, x(:, 3), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("Euler angle [rad]", 'Interpreter', 'latex');
legend("$\alpha$", "$\beta$", "$\gamma$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1, 3, 2);
plot(t, x(:, 4), "LineWidth", 2);
grid on 
hold on
plot(t, x(:, 5), "LineWidth", 2);
plot(t, x(:, 6), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$\omega_h$ [rad/s]", 'Interpreter', 'latex');
legend("$\omega_{hx}$", "$\omega_{hy}$", "$\omega_{hz}$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1, 3, 3);
plot(t, x(:, 7), "LineWidth", 2);
grid on 
hold on
plot(t, x(:, 8), "LineWidth", 2);
plot(t, x(:, 9), "LineWidth", 2);
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$\omega_w$ [rad/s]", 'Interpreter', 'latex');
legend("$\omega_{w1}$", "$\omega_{w2}$", "$\omega_{w3}$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);
