close all; clear all; clc;

addpath('Auxiliary/');

range      = [-5.2, 1.7, -2.05, 2.05];
gca_pos    = [0.02,0.05,0.87,0.9];
zoom_range = [-0.10, 0.10, -0.10, 0.10]*3.0;
cnt        = 69*3+1;
grid_size  = abs([range(2) - range(1), range(4) - range(3)]) / cnt;
[X, Y]     = meshgrid(linspace(range(1), range(2), cnt), linspace(range(3), range(4), cnt));
cnt_x2     = 20;
x2_bar     = 1.0;
x2_list    = x2_bar*[cos(linspace(0,2*pi,cnt_x2)); sin(linspace(0,2*pi,cnt_x2))];

p_o(:, 1) = [0;  1];
p_o(:, 2) = [0; -1];
R         = 0.99;
Radius    = 0.50;

p_o_circ  = [cos(linspace(0,2*pi,1000)); sin(linspace(0,2*pi,1000))];

[norm_original, norm_reshaped] = norm_of_solution_compare_new(X, Y, p_o, 1.0, exp(-R), [1; 0]);
norm_original(norm_original<0) = nan;
norm_reshaped(norm_reshaped<0) = nan;

figure(1); clf;
set(gcf, 'Position', [10, 10, 600, 300], 'color', 'w');
clrs = [[0.0, 0.0, 1.0]; [0.0, 0.0, 1.0]];
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal', 'position', gca_pos);
[H_original, h_colorbar] = plotColoredGrid_compare(X, Y, min(norm_original, 1), clrs(1, :), grid_size, 1.0, true);
h_colorbar.Label.String = {'Value of $|\rho_c(x_1)|$', '(darker = larger)'};
h_colorbar.Label.Interpreter = 'latex';
for i_o = 1:size(p_o, 2)
    H_obs1(i_o) = patch(p_o(1, i_o)+Radius*p_o_circ(1, :), p_o(2, i_o)+Radius*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'w', 'facealpha', 1.0, 'handlevisibility', 'off');
    H_obs1_plot(i_o) = plot3(p_o(1, i_o)+R*p_o_circ(1, :), p_o(2, i_o)+R*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'k', 'linewidth', 1);
    H_obs1(i_o).FaceColor = 'w';
    H_obs1(i_o).FaceAlpha = 1.0;
    H_obs1(i_o).ZData     = 2*ones(size(H_obs1(i_o).XData));
end

plot3(p_o(1, :), p_o(2, :), 3*ones(size(p_o(1,:))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text( p_o(1, 1)+0.1, p_o(2, 1), 3,                      '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text( p_o(1, 2)+0.1, p_o(2, 2), 3,                      '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[x_1]_1$', 'Interpreter','latex');
ylabel('$[x_1]_2$', 'Interpreter','latex');
axis(range)



hAxes = gca;
hNewAxes = copyobj(hAxes, gcf);
axes(hNewAxes); grid off;
axis(zoom_range); xticks([]); yticks([]);
set(hNewAxes, 'Position', [0.005, 0.1, 0.5, 0.8], 'fontsize',14, 'xtick', []);


figure(2); clf;
set(gcf, 'Position', [10, 10, 600, 300], 'color', 'w');
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal', 'position', gca_pos);
[H_reshaped, H_reshaped_colorbar] = plotColoredGrid_compare(X, Y, min(norm_reshaped, 1), clrs(2, :), grid_size, 0.7, true);
H_reshaped_colorbar.Label.String = {'Value of $|\rho_L(x_1)|$', '(darker = larger)'};
H_reshaped_colorbar.Label.Interpreter = 'latex';
for i_o = 1:size(p_o, 2)
    H_obs2(i_o) = patch(p_o(1, i_o)+Radius*p_o_circ(1, :), p_o(2, i_o)+Radius*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'w', 'facealpha', 1.0, 'handlevisibility', 'off');
    H_obs2_plot(i_o) = plot3(p_o(1, i_o)+R*p_o_circ(1, :), p_o(2, i_o)+R*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'k', 'linewidth', 1);
    H_obs2(i_o).FaceColor = 'w';
    H_obs2(i_o).FaceAlpha = 1.0;
    H_obs2(i_o).ZData     = 2*ones(size(H_obs2(i_o).XData));
end
plot3(p_o(1, :), p_o(2, :), 3*ones(size(p_o(1,:))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text( p_o(1, 1)+0.1, p_o(2, 1), 3,                      '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text( p_o(1, 2)+0.1, p_o(2, 2), 3,                      '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[x_1]_1$', 'Interpreter','latex');
ylabel('$[x_1]_2$', 'Interpreter','latex');
axis(range)

hAxes2 = gca;
hNewAxes2 = copyobj(hAxes2, gcf);
axes(hNewAxes2); grid off;
axis(zoom_range); xticks([]); yticks([]);
set(hNewAxes2, 'Position', [0.005, 0.1, 0.5, 0.8], 'fontsize',14, 'xtick', []);



