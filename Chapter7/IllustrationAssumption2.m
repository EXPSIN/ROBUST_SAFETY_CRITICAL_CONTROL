close all; clear all; clc;

addpath('Auxiliary/');

figure(1); 
subplot(1,2,1);
hold on;
box off;
grid off;
% axis off;
view(31, 10);


x_1_1  = linspace(-1.5, 2.5, 801);
x_1_2  = linspace(-1.5, 2.5, 801);
[X_1_1, X_1_2] = meshgrid(x_1_1, x_1_2);

xlim([min(x_1_1), max(x_1_1)]);

v_1 = 0.45;
v_2 = 0.15;
V_1 = exp(-(X_1_1+0).^2 - (X_1_2+0).^2 + 0.5.*X_1_1.*X_1_2)/1.2;
V_2 = exp(X_1_1+X_1_2-1)/exp(2)/2;
V_2 = exp(X_1_1+X_1_2-1)/exp(2)/2;

X      = (X_1_1+X_1_2-1 < 2.2 & (X_1_1~=0 | X_1_2~=0));
V_1(~X) = nan;
V_2(~X) = nan;


surf(X_1_1, X_1_2, V_1, get_clr(V_1, [0.0, 0.45, 0.74], 0.0), 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '$V_1(x_1)$');
surf(X_1_1, X_1_2, V_2, get_clr(V_2, [0.0, 0.80, 0.00], 0.0), 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '$V_2(x_1)$');

v_1_mat = v_1*ones(size(X_1_1));
v_2_mat = v_2*ones(size(X_1_1));
v_1_mat(~X) = nan;
v_2_mat(~X) = nan;
surf(X_1_1, X_1_2, v_1_mat, 'facecolor', [0.0, 0.45, 0.74], 'facealpha', 0.3, 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '${x_1\in\mathcal{X}:V_1(x_1)=v_1}$');
surf(X_1_1, X_1_2, v_2_mat, 'facecolor', [0.0, 0.80, 0.00], 'facealpha', 0.3, 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '${x_1\in\mathcal{X}:V_2(x_1)=v_2}$');


% xlabel('$[x_1]_1$', 'Interpreter','latex');
% ylabel('$[x_1]_2$', 'Interpreter','latex');
% zlabel('$V_j(x_1)$', 'Interpreter','latex');

% legend('Interpreter','latex', 'Location', 'northwest');
set(gcf, 'position', [50, 50, 800,400], 'color', 'w');
set(gca, 'Position', [0.05, 0.10, 0.45, 0.95], 'FontSize', 20);
xticks([])
yticks([])
zticks([])

subplot(1,2,2);
hold on;
box off;
grid off;
axis equal;
view(2);
xlim([-1.5, max(x_1_1)]);
ylim([-1.5, max(x_1_2)]);

V_1_unsafe = V_1;
V_2_unsafe = V_2;
V_1_unsafe(V_1<v_1) = nan;
V_2_unsafe(V_2<v_2) = nan;


surf(X_1_1, X_1_2, V_1_unsafe, 'facecolor', [0.0, 0.45, 0.74], 'facealpha', 0.5, 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '$V_1(x_1)$');
surf(X_1_1, X_1_2, V_2_unsafe, 'facecolor', [0.0, 0.80, 0.00], 'facealpha', 0.5, 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '$V_2(x_1)$');

X_mat     = -ones(size(X_1_1));
notX_mat  = X_mat;
X_mat(~X) =  nan;
notX_mat(X)  =  nan;

surf(X_1_1, X_1_2, X_mat, 'facecolor', 0.90*[1, 1, 1], 'facealpha', 0.3, 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '${x_1\in\mathcal{X}:V_1(x_1)=v_1}$');
surf(X_1_1, X_1_2, notX_mat, 'facecolor', 0.0*[1, 1, 1], 'facealpha', 1.0, 'linestyle', 'none', 'LineWidth', 2, 'DisplayName', '${x_1\in\mathcal{X}:V_1(x_1)=v_1}$');

% xlabel('$[x_1]_1$', 'Interpreter','latex');
% ylabel('$[x_1]_2$', 'Interpreter','latex');
% zlabel('$V_j(x_1)$', 'Interpreter','latex');

% legend('Interpreter','latex', 'Location', 'northwest');
set(gcf, 'position', [50, 50, 800,400], 'color', 'w');
set(gca, 'Position', [0.50, 0.10, 0.45, 0.80], 'FontSize', 20, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xticks([])
yticks([])
zticks([])

function clr_V = get_clr(V, clr, bias)
[m, n] = size(V);
v_max = max(V(:));
v_min = min(V(:));
clr_V = zeros(m, n, 3);
for i = 1:m
    for j = 1:n
        k_ratio = min(max((V(i,j)-v_min)/v_max+bias, 0), 1);
        clr_V(i,j,:) = k_ratio*clr+(1-k_ratio)*[1,1,1];
    end
end
end