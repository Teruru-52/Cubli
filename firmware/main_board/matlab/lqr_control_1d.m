clear all
close all

dt = 0.01;
coeff_mb = 1.0;
coeff_mw = 0.2;
coeff_Ib = 0.1;
coeff_Iw = 0.1;
coeff_Lb = 0.1;
coeff_Lw = 0.05;
coeff_cb = 0.001;
coeff_cw = 0.001;
coeff_k_tau = 0.1;
coeff_g = 9.81;
%% Linearlized Model (1D)
syms theta_b dot_theta_b dot_theta_w
syms mb mw Ib Iw Lb Lw cb cw k_tau g

f = [dot_theta_b;
      ((mb*Lb+mw*Lw)*g*sin(theta_b) + cw*dot_theta_w - cb*dot_theta_b) / (Ib+mw*Lw^2);
      -(Ib+Iw+mw*Lw^2)*cw*dot_theta_w / (Iw*(Ib+mw*Lw^2)) - ((mb*Lb+mw*Lw)*g*sin(theta_b)-cb*dot_theta_b) / (Ib+mw*Lw^2)];

dfdx = [diff(f(1), theta_b), diff(f(1), dot_theta_b), diff(f(1), dot_theta_w);
            diff(f(2), theta_b), diff(f(2), dot_theta_b), diff(f(2), dot_theta_w);
            diff(f(3), theta_b), diff(f(3), dot_theta_b), diff(f(3), dot_theta_w)];

xd = [0; 0; 0];
% linearized system matrix and input matrix
A = subs(dfdx, [theta_b, dot_theta_b, dot_theta_w], [xd(1) xd(2) xd(3)])
B = [0;
       -k_tau / (Ib+mw*Lw^2);
       k_tau*(Ib+Iw+mw*Lw^2) / (Iw*(Ib+mw*Lw^2))]

A = subs(A, [mb, mw, Ib, Iw, Lb, Lw, cb, cw, g], [coeff_mb, coeff_mw, coeff_Ib, coeff_Iw, coeff_Lb, coeff_Lw, coeff_cb, coeff_cw, coeff_g])
B = subs(B, [mw, Ib, Iw, Lw, k_tau], [coeff_mw, coeff_Ib, coeff_Iw, coeff_Lw, coeff_k_tau])
%% Simulation (1D)
A = [0    1.0000         0;
    10.7373   -0.0100    0.0100;
    -10.7373    0.0100   -0.0200];
B = [0;  -0.9950; 1.995];

V = ctrb(A, B);
rank(V)

Q = diag([10, 10, 10]);
R = 1;

[F, P, E] = lqrd(A, B, Q, R, dt);
F = -F

% simulation
tspan = [0:dt:10];
x0 = [0.2; 0.3; 0.1];
[t, x] = ode45(@(t, x) (A+B*F)*x, tspan, x0);

figure(1);
subplot(1, 2, 1);
plot(t, x(:, 1), "LineWidth", 2);
grid on
xlabel("time [s]", 'Interpreter', 'latex');
hold on
plot(t, x(:, 2), "LineWidth", 2);
ylabel("$\theta_b$ [rad], $\dot\theta_b$ [rad/s]", 'Interpreter', 'latex');
legend("$\theta_b$", "$\dot\theta_b$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(1, 2, 2);
plot(t, x(:, 3), "LineWidth", 2);
grid on
xlabel("time [s]", 'Interpreter', 'latex');
ylabel("$\theta_\omega$ [rad/s]", 'Interpreter', 'latex');
legend("$\theta_\omega$", 'Interpreter', 'latex')
set(gca, "FontName", "Times New Roman", "FontSize", 15);