clear all
close all

%% Matrix for Gravity Vector Estimation
l = 0.15; % [m]
px1 = 0.0025 + 0.00157;
py1 = 0.033;
pz1 = 0.0115;

p1 = [px1; py1; pz1];
p2 = [l - py1; pz1; px1];
p3 = [l - pz1; l - px1; py1];
p4 = [pz1; px1; l - py1];
p5 = [py1; l - pz1; l - px1];
p6 = [l - px1; l - py1; l - pz1];

% p1 = [0.008; 0.033; 0.003];
% p2 = [0.113; 0.003; 0.008];
% p3 = [0.143; 0.113; 0.008];
% p4 = [0.138; 0.033; 0.143];
% p5 = [0.033; 0.143; 0.138];
% p6 = [0.113; 0.143; 0.138];

P = cat(1, ones(1, 6), [p1 p2 p3 p4 p5 p6]);
X = pinv(P);
X(:, 1)'

%% plot
vertices = l * [
    0, 0, 0;     % 頂点1
    1, 0, 0;     % 頂点2
    1, 1, 0;     % 頂点3
    0, 1, 0;     % 頂点4
    0, 0, 1;     % 頂点5
    1, 0, 1;     % 頂点6
    1, 1, 1;     % 頂点7
    0, 1, 1;     % 頂点8
];

faces = [
    1, 2, 3, 4;  % 底面
    5, 6, 7, 8;  % 上面
    1, 2, 6, 5;  % 側面1
    2, 3, 7, 6;  % 側面2
    3, 4, 8, 7;  % 側面3
    4, 1, 5, 8   % 側面4
];

p = [p1 p2 p3 p4 p5 p6];

% adjust quiver length
norm = 1;
text_labels = {1, 2, 3, 4, 5, 6};
dist = 0.005;

figure(1);
for i = 1:6
    quiver3(0, 0, 0, p(1,i), p(2,i), p(3,i),0, 'LineWidth', 1.5);
    hold on;
    text(p(1,i)+dist, p(2,i)+dist, p(3,i)+dist, text_labels(i), "FontName","Times New Roman", 'FontSize', 20);
end
patch('Vertices', vertices, 'Faces', faces, 'Facecolor', 'none', 'Linewidth', 1.5);
daspect([1 1 1]);
grid on;
legend("imu1", "imu2", "imu3", "imu4", "imu5", "imu6");

xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
zlabel('$z$ [m]', 'Interpreter', 'latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);
